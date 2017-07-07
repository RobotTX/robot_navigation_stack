#include "recover_position.hpp"

using boost::asio::ip::tcp;

#define ROBOT_POS_TOLERANCE 0.5

bool waitingForNextGoal = false;
coordinates currentGoal;

ros::ServiceClient sendPositionRecoveredConfirmation;

std::vector<int8_t> my_map;
Metadata metadata;
RobotPos robotOrigin;

geometry_msgs::Pose robot_full_pos;

/// to process the goals needed to recover the position
std::shared_ptr<MoveBaseClient> ac(0);

ros::Subscriber localizationToolFeedbackSubscriber;
ros::Subscriber amclPoseSubscriber;

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	my_map.clear();
	for(size_t i = 0; i < msg->info.width * msg->info.height; i++)
		my_map.push_back(msg->data[i]);
}

void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg){
	metadata.resolution = msg->resolution;
	metadata.width = msg->width;
	metadata.height = msg->height;
	metadata.x = msg->origin.position.x;
	metadata.y = msg->origin.position.y;
}

void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg){
	robotOrigin.x = msg->position.x;
	robotOrigin.y = msg->position.y;

	/// used later to send the recovered position to the application
	robot_full_pos = *msg;

	/// if a goal has been set, we check if it's been reached
	if(currentGoal.first != -1){
		/// we check if the robot is close enough to its goal
		if(std::abs(msg->position.x - currentGoal.first) < ROBOT_POS_TOLERANCE && std::abs(msg->position.y - currentGoal.second) < ROBOT_POS_TOLERANCE){
			/// if the robot has already arrived, we want to wait for the next goal instead of repeat the same "success" functions
			if(!waitingForNextGoal){
				std::cout << "(PlayPath) getRobotPos robot close enough to the goal" << std::endl;
				std::cout << "(PlayPath) robot position " << msg->position.x << " " << msg->position.y 
				<< "\n(PlayPath) robot goal " << currentGoal.first << " " << currentGoal.second
				<< std::endl;
				waitingForNextGoal = true;
				currentGoal.first = -1;
			}
		}
	}
}

void checkRecoveryStatus(const std_msgs::String& msg){

	/// the position has not been found yet
	if(msg.data != "position found") {
		// need to find a way to make sure the goal has been reached using the function in play_path
		if(waitingForNextGoal) 
			findNextPoint();
	} else {
		/// since we have found the position we cancel the goal using cancelAllGoals (does not crash if no goal was sent)
		ac->cancelAllGoals();
		currentGoal.first = -1;
		/// we tell the application the position was recovered
		gobot_software::RecoveredPosition srv;
		if(sendPositionRecoveredConfirmation.call(srv))
			std::cout << "Service send position recovered confirmation called" << std::endl;
		else
			std::cout << "Service send position recovered confirmation could not be called" << std::endl;
	}
}

void checkEstimatedPosition(const geometry_msgs::PoseWithCovarianceStamped& msg){
	// we check if the covariance of the position is within our expectations (the lower the covariance the more amcl is sure about the position of the robot)
	// if any of the diagonal values is more than our threshold we return otherwise we can notify the application that the position was recovered
	std::cout << "Current estimated pos " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << std::endl;
	for(uint i = 0; i < 6; i++){
		std::cout << " covariance diag " << i << " " << msg.pose.covariance[i * 6 + i] << std::endl;
		if(msg.pose.covariance[i * 6 + i] > 1) return;
	}
	// TODO call a service
	gobot_software::RecoveredPosition position;
	position.request.x = msg.pose.pose.position.x;
	position.request.y = msg.pose.pose.position.y;
	position.request.z = msg.pose.pose.orientation.z;
	position.request.w = msg.pose.pose.orientation.w;
	sendPositionRecoveredConfirmation.call(position);
}

bool recoverPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(RECOVER POSITION) recoverPosition service called";
	
	ros::NodeHandle n;

	localizationToolFeedbackSubscriber = n.subscribe("position_found", 1, checkRecoveryStatus);

	amclPoseSubscriber = n.subscribe("/amcl_pose", 1, checkEstimatedPosition);

	return true;
}

bool stopRecoveringPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	
	localizationToolFeedbackSubscriber.shutdown();
	
	return true;
}

ReducedMap reduceMap(const std::vector<int8_t>& _map, const uint32_t width, const uint32_t height){

	ReducedMap reducedMap;

	for(size_t i = 0; i < height; i++){
		std::vector<Sequence> row;
		Sequence sequence = (_map[i*width] < 30 && _map[i*width] != -1) ? Sequence ({ Color::White, 0, 0 }) : Sequence ({ Color::Black, 0, 0 }); 
		for(size_t j = 1; j < width; j++){
			// this is a white point
			if(_map[i*width + j] < 30 && _map[i*width +j] != -1){
				// this is a sequence of white points so we update the end of the sequence
				if(sequence.color == Color::White)
					sequence.end++;
				// this is a sequence of unknown points so we push the current sequence and start a new sequence of white points
				else {
					row.push_back(sequence);
					sequence.color = Color::White;
					sequence.start = j;
					sequence.end = j;
				}
				
			} else {
				// this is a sequence of white points so we push the current sequence and start a new sequence of black points
				if(sequence.color == Color::White){
					row.push_back(sequence);
					sequence.color = Color::Black;
					sequence.start = j;
					sequence.end = j;
				} 
				// this is a sequence of black points so we update the end of the sequence
				else 
					sequence.end++;
			}
		}
		row.push_back(sequence);
		reducedMap.push_back(row);
	}

	return reducedMap;
}

void addEdgeToReducedGraph(const ReducedVertex head, const ReducedVertex tail, reducedGraph_t & graph){
	graph[head].insert(tail);
	graph[tail].insert(head); 
}

void graphFromReducedMap(const ReducedMap& _map, reducedGraph_t & graph, const int minSpaceRequired){
	// there is no need to look for possible edges at the last row since there is no potential "neighbors" below it, hence the "-1" in _map.size()-1
	for(int i = 0; i < _map.size()-1; i++){
		for(size_t j = 0; j < _map[i].size(); j++){
			Sequence currSequence = _map[i][j];
			// if the current is large enough ( for the robot to move there )
			if(currSequence.color == Color::White && (currSequence.end - currSequence.start + 1) >= minSpaceRequired){
				// we check if there is another sequence that is large enough below us, because the graph is undirected it is not necessary
				// to check whether or not there is such a sequence above ( basically we are constructing the graph from top to bottom )
				for(size_t k = 0; k < _map[i+1].size(); k++){
					Sequence currSeqBelow = _map[i+1][k];
					// if the sequence is a sequence of white points this is a potential "neighbor"
					if(currSeqBelow.color == Color::White){
						if(currSeqBelow.start <= currSequence.end && currSeqBelow.end >= currSequence.start // this is checking if the sequences are somewhat aligned 
						&& std::min(currSeqBelow.end, currSequence.end) - std::max(currSeqBelow.start, currSequence.start) + 1 >= minSpaceRequired){ // this is checking if there is enough space for the robot to pass 
							addEdgeToReducedGraph(ReducedVertex({currSequence.start, currSequence.end, i}), ReducedVertex({currSeqBelow.start, currSeqBelow.end, i+1}), graph);
						}
					}
				}
			}
		}
	}
}

/* this function assumes that the position given for the robot allows it to move at all */
std::pair<ReducedVertex, int> findFurthestPointInReducedGraph(const RobotPos& robotOrigin, const Metadata& _map, 
	const reducedGraph_t& graph, const ReducedMap& _reducedMap){
	coordinates originInPixels = std::make_pair(
						(-_map.x + robotOrigin.x) / _map.resolution + ROBOT_WIDTH,
						_map.height-(-_map.y + robotOrigin.y) / _map.resolution - ROBOT_WIDTH/2
	);

	ReducedVertex origin = reducedVertexFromPixels(originInPixels, _reducedMap);	

	// if the robot cannot move from where it is
	if(graph.find(origin) == graph.end())
		return std::make_pair(ReducedVertex({-1, -1, -1}), -1);

	std::pair<ReducedVertex, int> furthestPoint({-1, -1, -1}, 0);
	std::queue<ReducedVertex> _queue;
	std::map<ReducedVertex, int, compReducedVertices> distances;
	for(auto it = graph.cbegin(); it != graph.cend(); ++it)
		distances[(*it).first] = std::numeric_limits<int>::max();

	distances[origin] = 0;
	_queue.push(origin);

	while(!_queue.empty()){
		ReducedVertex current = _queue.front();
		_queue.pop();
		for(auto it = graph.at(current).cbegin(); it != graph.at(current).cend(); ++it){
			const ReducedVertex next = *it;
			
			if(distances[current] + abs(next.end-current.start) + 1 < distances[next]){
				distances[next] = distances[current] + abs(next.end-current.start) + 1;
				_queue.push(next); 
			}
			
		}
	}
	for(auto it = distances.cbegin(); it != distances.cend(); ++it){
		// if this point is further than the furthest point we have found so far we update the furthest point 
		if((*it).second > furthestPoint.second && (*it).second != std::numeric_limits<int>::max()){
			furthestPoint.second = (*it).second;
			furthestPoint.first.row = (*it).first.row;
			furthestPoint.first.start = (*it).first.start;
			furthestPoint.first.end = (*it).first.end;
		}
	}
	return furthestPoint;
}

ReducedVertex reducedVertexFromPixels(const coordinates pixelsCoordinates, const ReducedMap& _map){
	for(size_t i = 0; i < _map[pixelsCoordinates.second].size(); i++){
		if(_map[pixelsCoordinates.second][i].start <= pixelsCoordinates.first && _map[pixelsCoordinates.second][i].end >= pixelsCoordinates.first)
			return ReducedVertex({_map[pixelsCoordinates.second][i].start, _map[pixelsCoordinates.second][i].end, pixelsCoordinates.second});
	}
}

bool findNextPoint(){

	std::cout << metadata.height << " " << metadata.width << " " << my_map.size() << robotOrigin.x << "" << robotOrigin.y << std::endl;
	if(metadata.height != 0 && metadata.width != 0 && my_map.size() == metadata.width * metadata.height){
		ReducedMap reducedMap = reduceMap(my_map, metadata.width, metadata.height);
		
		
		reducedGraph_t graph;

		graphFromReducedMap(reducedMap, graph, 8);

		std::pair<ReducedVertex, int> furthestPoint = findFurthestPointInReducedGraph(robotOrigin, metadata, graph, reducedMap);
		std::cout << furthestPoint.first.row << std::endl;
		if(furthestPoint.first.row != -1){
		
			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "/map1";
		    goal.target_pose.header.stamp = ros::Time::now();
		    goal.target_pose.pose.position.x = (furthestPoint.first.end) * metadata.resolution + metadata.x;
		    goal.target_pose.pose.position.y = (furthestPoint.first.row) * metadata.resolution + metadata.y;
		    goal.target_pose.pose.orientation.x = 0;
		    goal.target_pose.pose.orientation.y = 0;
		    goal.target_pose.pose.orientation.z = 0;
		    goal.target_pose.pose.orientation.w = 1;

		    currentGoal = std::make_pair((furthestPoint.first.end) * metadata.resolution + metadata.x, (furthestPoint.first.row) * metadata.resolution + metadata.y);

		    std::cout << "Only a test otherwise I would send this goal " << goal.target_pose.pose.position.x << " " << goal.target_pose.pose.position.y << std::endl;
			// TODO uncomment 
			//ac->sendGoal(goal);
		}
	}
	return true;
}

int main(int argc, char* argv[]){

	std::cout << "(Recover position) recover position main running..." << std::endl;

	try {

		ros::init(argc, argv, "recover_position");

		ros::NodeHandle n; 

		// to receive the metadata of the map : width, height, resolution and origin
		ros::Subscriber sub_metadata = n.subscribe("/map_metadata", 1000, getMetaData);

		// to receive the robot's position and orientation
		ros::Subscriber sub_robot = n.subscribe("/robot_pose", 1, getRobotPos);
		
		// to receive the current map
		ros::Subscriber sub_map = n.subscribe("/map", 1000, getMap);

		// tell the action client that we want to spin a thread by default
		ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));

		// to send the position of the robot to the application once recovered
		ros::ServiceServer recover_position_service = n.advertiseService("recover_position", recoverPosition);

		ros::ServiceServer stop_recovering_position_service = n.advertiseService("stop_recovering_position", stopRecoveringPosition);

		sendPositionRecoveredConfirmation = n.serviceClient<gobot_software::RecoveredPosition>("send_position_recovered_confirmation");
		
		// wait for the action server to come up
		while(!ac->waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for the move_base action server to come up");

		if(ac->isServerConnected())
                        std::cout << "Recover position:: Server is connected" << std::endl;

		ros::spin();

	} catch (std::exception& e) {

		std::cout << "Recover position exception " << e.what() << std::endl;
	}
	
	return 0;
}
