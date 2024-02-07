/**
 * the file contains the implementation of all functions
 * defined in the ur5_task_planner_library.h file
 */
#include "ur5/ur5_task_library.h"

void gestisciStato(stato &state,ros::NodeHandle n){
    switch (state){
        case start:{
            //ask the 3d cam for the blocks' position
            //at the moment, as there is no vision node, we manually
            //initialize the pose of each block
        
            //X1-Y2-Z1
            //testing if block is unreachable
            //pos[0]<< 3.85, 0.7, 0.87;
            pos[0]<< 0.85, 0.7, 0.87;
            phi[0]<< 0, 0, 0;
            //X1-Y2-Z2-TWINFILLET
            pos[1]<< 0.7, 0.7, 0.87;
            phi[1]<< 0, 0, 0.785398;
            //X1-Y3-Z2-FILLET
            pos[2]<< 0.85, 0.5, 0.87;
            phi[2]<< 0, 0, 0;
            //X1-Y4-Z1
            pos[3]<< 0.7, 0.5, 0.87;
            phi[3]<< 0, 0, 1.57;
            //X1-Y4-Z2
            pos[4]<< 0.9, 0.3, 0.87;
            phi[4]<< 0, 0, 1.57;
            
            //set the class for each block between 1 and 5 (6 for no possible grasping)
            class_of_block[0]=1;
            class_of_block[1]=2;
            class_of_block[2]=3;
            class_of_block[3]=4;
            class_of_block[4]=5;

            //convert coordinates from world to robot frame
            for(int i=0;i<n_classes;i++){
                worldToRobotFrame(pos[i],phi[i]);
            }
            
            ROS_INFO("block_request");
            state=block_request;

            break;
        }

        case block_request:{
            //requesting pose/class of a block
            
            //get "k" block values
            for(int i=0;i<3;i++){
                xef[i]=pos[k](i);
                phief[i]=phi[k](i);
            }

            //basing on class block, get the known final position
            switch (class_of_block[k]){
                case 1:{
                    //X1-Y2-Z1
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.32,GRASPING_HEIGHT;
                                       
                    break;
                }
                case 2:{
                    //X1-Y2-Z2-TWINFILLET
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.40,GRASPING_HEIGHT;
                   
                    break;
                }
                case 3:{
                    //X1-Y3-Z2-FILLET
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.48,GRASPING_HEIGHT;
                    
                    break;
                }
                case 4:{
                    //X1-Y4-Z1
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.56,GRASPING_HEIGHT;
                    
                    break;
                }
                case 5:{
                    //X1-Y4-Z2
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.64,GRASPING_HEIGHT;              

                    break;
                }
                case 6:{
                    //X2-Y2-Z2
                    //not used because of bad meshes
                    phief_class<<0,0,M_PI/2;
                    xef_class<<0.1,0.72,GRASPING_HEIGHT;

                    break;
                }
            }

            //convert from world to robot frame
            worldToRobotFrame(xef_class,phief_class);

            if(final_end==0){
                //still blocks
                ROS_INFO("high_block_take");
                state=high_block_take;
            }   
            else{
                //block finished to move
                ROS_INFO("no_more_blocks");
                state=no_more_blocks;
            }
            break;
        }

        case high_block_take:{
            gripper=GRIPPER_OPEN; 
            xef[2]=SAFE_Z_MOTION;
    
            //request to motion service
            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv));//wait until response 
            error=srv.response.error;

            switch(error){
                    case 0:{
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        ROS_INFO("block_take");
                        state=block_take;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        ROS_INFO("block_request");
                        state=block_request;
                        ++k;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        ROS_INFO("motion_error");
                        //next_state=block_take;
                        next_state=high_block_take;
                        state=motion_error;
                        break;

                    }
                }         

            break;
        }

        case block_take:{ 
            gripper=GRIPPER_OPEN;
            xef[2]=WORLD_TO_ROBOT_Z - GRASPING_HEIGHT;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        ROS_INFO("high_block_release");
                        state=high_block_release;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        ROS_INFO("block_request");
                        state=block_request;
                        ++k;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        ROS_INFO("motion_error");
                        //next_state=high_block_release;
                        next_state=block_take;
                        state=motion_error;
                        break;

                    }
                }         

            break;
        }
        
        case high_block_release:{
            gripper=GRIPPER_CLOSE;
            xef[2]=SAFE_Z_MOTION;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        ROS_INFO("high_class_release");
                        state=high_class_release;
                        break;
                    }
                    case 1:{srv.request.ack=0;
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        ROS_INFO("block_request");
                        state=block_request;
                        ++k;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        ROS_INFO("motion_error");
                        //next_state=high_class_release;
                        next_state=high_block_release;
                        state=motion_error;
                        break;

                    }
                }         

            break;

        }

        case high_class_release:{
            //set block final pose as destination
            for(int i=0;i<3;i++){
                phief[i]=phief_class(i);
                xef[i]=xef_class(i);
                
            }

            gripper=GRIPPER_CLOSE;
            xef[2]=SAFE_Z_MOTION;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        ROS_INFO("class_release");
                        state=class_release;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        ROS_INFO("block_request");
                        state=block_request;
                        ++k;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        ROS_INFO("motion_error");
                        //next_state=class_release;
                        next_state=high_class_release;
                        state=motion_error;
                        break;

                    }
                }         

            break;

        }

        case class_release:{
            gripper=GRIPPER_CLOSE;
            xef[2]=WORLD_TO_ROBOT_Z - GRASPING_HEIGHT;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
            
                
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                    case 0:{
                        ROS_INFO("Motion planning executed correctly!\n\n"); 
                        ROS_INFO("high_class_take");
                        state=high_class_take;
                        break;
                    }
                    case 1:{
                        ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                        ROS_INFO("block_request");
                        state=block_request;
                        ++k;
                        break;
                    }
                    case 2:{
                        ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                        ROS_INFO("motion_error");
                        //next_state=high_class_take;
                        next_state=class_release;
                        state=motion_error;
                        break;

                    }
                }         

            break;
        }

        case high_class_take:{
            gripper=GRIPPER_OPEN;
            xef[2]=SAFE_Z_MOTION;

            //check if this block is the last to be moved
            if(k++ == n_classes-1) final_end=1;

            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;

            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=xef[0];
            srv.request.xef2=xef[1];
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=0;
            srv.request.ack=0;
            
            while(!service.call(srv)); 
            error=srv.response.error;

            switch(error){
                case 0:{
                    ROS_INFO("Motion planning executed correctly!\n\n"); 
                    if(final_end!=1){ROS_INFO("block_request"); state=block_request;}
                    else {ROS_INFO("no_more_blocks"); state=no_more_blocks;}
                    break;
                }
                case 1:{
                    ROS_INFO("UNREACHABLE POSITION: move on to the next block\n\n");
                    ROS_INFO("block_request");
                    state=block_request;
                    ++k;
                    break;
                }
                case 2:{
                    ROS_INFO("ERROR IN MOTION: move on to a different planning (passing through an intermediate safe point)\n\n");
                    if(final_end!=1){ROS_INFO("block_request"); state=block_request;}
                    else {ROS_INFO("no_more_blocks"); state=no_more_blocks;}
                    state=motion_error;
                    break;

                }
            }         

            break;
        }
        
        case motion_error:{
            //go through safe point in case of motion errors
            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;
            srv.request.phief1=phief[0];
            srv.request.phief2=phief[1];
            srv.request.phief3=phief[2];

            srv.request.xef1=0;
            srv.request.xef2=-0.4;
            srv.request.xef3=xef[2];
            srv.request.gripper=gripper;
            srv.request.end=final_end;
            srv.request.ack=0;
   
            while(!service.call(srv)); 
            error=srv.response.error;
            if(error == 2) exit(1);
            state=next_state;
             
            ROS_INFO("going to next_state");
            break;
        }
        
        case no_more_blocks:{
            //this is the last state, terminate the node
            //move the manipulator to a safe final position
            ros::ServiceClient service = n.serviceClient<ur5::ServiceMessage>("tp_mp_communication");
            ur5::ServiceMessage srv;
            
            Vector3d xef, phief;
            xef << 0.5, 0.7, 1.3;
            phief << 0, 0, 0;

            //convert from world to robot frame
            worldToRobotFrame(xef,phief);

            srv.request.phief1=phief(0);
            srv.request.phief2=phief(1);
            srv.request.phief3=phief(2);

            srv.request.xef1=xef(0);
            srv.request.xef2=xef(1);
            srv.request.xef3=xef(2);
            srv.request.gripper=0;
            srv.request.end=1;
            srv.request.ack=0;
            
            service.call(srv);

            ROS_INFO("All blocks moved");
            next_state = no_more_blocks;
            
            break;
        }
    }
}

void worldToRobotFrame(Vector3d& coords, Vector3d& euler){
    Vector4d position;
    position << coords[0], coords[1], coords[2], 1;

    coords = (WORLD_TO_ROBOT*position).block(0,0,3,1);
    coords*=SCALAR_FACTOR;

    euler(1) = -euler(1);
    euler(2) = -euler(2);
}