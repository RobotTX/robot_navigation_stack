#ifndef BACKUP_SYSTEM
#define BACKUP_SYSTEM

#include "ros/ros.h"
#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/regex.hpp>

#define CONNECTION_PORT 6498


using boost::asio::ip::tcp;

void server(const unsigned short port);
void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor);
void sendMessageToApplication(boost::shared_ptr<tcp::socket> socket, const std::string message);
void session(boost::shared_ptr<tcp::socket> socket);
void disconnect(void);
void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif