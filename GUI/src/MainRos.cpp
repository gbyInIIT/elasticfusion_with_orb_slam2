/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "MainControllerRos.h"
#include <zmq.hpp>
#include <string>
#include <iostream>

int main(int argc, char * argv[])
{
//    MainControllerRos mainController(argc, argv);
//    MainControllerRos * mainControllerPtr = NULL;
//    mainControllerPtr = new MainControllerRos(argc, argv);
////    auto quitAction = [&mainController, &mainControllerPtr]() {
//    auto quitAction = [&mainControllerPtr]() {
//        usleep(20000000);
//        printf("Quit window.\n");
//        fflush(stdout);
////        mainController.isMainControllerRunning.assign(false);
//        mainControllerPtr->isMainControllerRunning.assign(false);
//    };
//    std::thread quitThread = std::thread(quitAction);
////    mainController.launch();
//    mainControllerPtr->launch();
//    quitThread.join();
//    delete(mainControllerPtr);

    //  Prepare our context and socket
    MainControllerRos * mainControllerPtr = NULL;
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);
    socket.bind ("tcp://*:5557");
    auto newEfusionAction = [&mainControllerPtr, &argc, &argv] () {
        mainControllerPtr = new MainControllerRos(argc, argv);
        mainControllerPtr->launch();
        delete(mainControllerPtr);
        mainControllerPtr = NULL;
    };
    std::thread mainControllerThread;
    while (true) {
        zmq::message_t request;

        //  Wait for next request from client
        socket.recv (&request);
        std::cout << "Received Hello" << std::endl;
        if (mainControllerPtr) {
            mainControllerPtr->isMainControllerRunning.assign(false);
            mainControllerThread.join();
//            mainControllerPtr = NULL;
        } else {
            mainControllerThread = std::move(std::thread(newEfusionAction));
//            mainControllerThread.detach();
        }

        //  Do some 'work'
        sleep(1);

        //  Send reply back to client
        zmq::message_t reply (5);
        memcpy (reply.data (), "World", 5);
        socket.send (reply);
    }
    return 0;
}
