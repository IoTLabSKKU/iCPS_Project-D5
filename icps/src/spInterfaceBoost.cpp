#include <ctime>
#include <iostream>
#include <string>
#include <thread>
#include <list>

#include <syslog.h>
#include <signal.h>
#include <future>
#include "ROSHelper.h"
#include "spInterfaceConnection.h"
#include "ServerTemplate.h"

class spInterfaceServer: public ServerTemplate<spInterfaceConnection>{
public:
	using ServerTemplate::ServerTemplate;
};

void spinThreadHandler(){
	ros::spin();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "spInterfaceBoost");

    //handle option
	string serverIP = "127.0.0.1";
	string serverPort = "6278";
	int option;
	int threadNum = 5;

	while ((option = getopt(argc, argv, "s:p:n:")) != -1){
		INFMSG("option %c is set to %s\n", option, optarg);

		switch(option){
		case 's':
			serverIP = optarg;
			break;
		case 'p':
			serverPort = optarg;
			break;
		case 'n':
			threadNum = atoi(optarg);
			break;
		}
	}

	INFMSG("server: %s:%s\n", serverIP.c_str(), serverPort.c_str());
	INFMSG("ThreadNum: %d\n\n", threadNum);



	try{
		boost::thread spinThread(&spinThreadHandler);

		int sig = 0;
		sigset_t new_mask;
		sigset_t old_mask;
		sigfillset(&new_mask);
		pthread_sigmask(SIG_BLOCK, &new_mask, &old_mask);

		spInterfaceServer server(serverIP, serverPort, threadNum);
		std::packaged_task<int()> server_task([&]()->int{server.run();return 0;});
		auto future = server_task.get_future();
		std::thread server_thread(std::move(server_task));
		INFMSG("Waiting for connection...\n");

		sigset_t wait_mask;
		pthread_sigmask(SIG_SETMASK, &old_mask, nullptr);
		sigemptyset(&wait_mask);
		sigaddset(&wait_mask, SIGINT);
		sigaddset(&wait_mask, SIGQUIT);
		sigaddset(&wait_mask, SIGTERM);
		pthread_sigmask(SIG_BLOCK, &wait_mask, nullptr);
		sigwait(&wait_mask, &sig);

		server.stop();
		auto status = future.wait_for(std::chrono::seconds(2));
		if(status == std::future_status::ready){
			server_thread.join();
		    INFMSG("Exit Daemon!\n");
		}else{
			ERRMSG("Hard abort serverTaskThread\n");
			server_task.reset();
		}

		spinThread.join();
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}

	return 0;
}

