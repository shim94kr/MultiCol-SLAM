// The original version was released under the following license
/**
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

// All modifications are released under the following license
/**
 * This file is part of MultiCol-SLAM
 *
 * Copyright (C) 2015-2016 Steffen Urban <rurbste at googlemail.com>
 * For more information see <https://github.com/urbste/MultiCol-SLAM>
 *
 * MultiCol-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MultiCol-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MultiCol-SLAM . If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <string>

#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

#include "cTracking.h"
#include "cConverter.h"
#include "cam_model_omni.h"
#include "cSystem.h"
#include "capture1CH.h"

using namespace std;
using namespace cv;

void LoadImagesAndTimestamps(
                const int startFrame,
                const int endFrame,
                const string path2imgs,
                vector<vector<string>> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
        if (argc != 5)
        {
                cerr << endl << "Usage: ./MultiCol_Slam_Lafida vocabulary_file slam_settings_file path_to_settings path_to_img_sequence" << endl;
                return 1;
        }

        string path2voc = string(argv[1]);
        string path2settings = string(argv[2]);
        string path2calibrations = string(argv[3]);
        string path2imgs = string(argv[4]);

        cout << endl << "MultiCol-SLAM Copyright (C) 2016 Steffen Urban" << endl << endl;
        socketOpen();
	
        //pthread_mutex_init(&gmutex, NULL);
        //pthread_cond_init(&gcond, NULL);
	
	recv_buffer[0] = (char*)malloc(rec_max_size);
	recv_buffer[1] = (char*)malloc(rec_max_size);
	std::thread thread0(threadFcn0);
	std::thread thread1(threadFcn1);
	// p_thread[0] = new thread(threadFcn0);
	// p_thread[1] = new thread(threadFcn1);
	char start;
        scanf("%c",&start);
        /*if(start == 's') {
                if ( (thr_id_0 = pthread_create(&p_thread[0], NULL, threadFcn0, (void *)&a)) <0)
                        printf("thread 1 error");
                if ( (thr_id_1 = pthread_create(&p_thread[1], NULL, threadFcn1, (void *)&a)) < 0)
                        printf("thread 2 error");
        }*/
        // --------------
        // 1. Tracking settings
        // --------------
        cv::FileStorage frameSettings(path2settings, cv::FileStorage::READ);

        int traj = (int)frameSettings["traj2Eval"];
        string trajs = to_string(traj);
        const int endFrame = (int)frameSettings["traj.EndFrame"];
        const int startFrame = (int)frameSettings["traj.StartFrame"];

        // --------------
        // 4. Load image paths and timestamps
        // --------------
        vector<vector<string>> imgFilenames;
        vector<double> timestamps;
	printf ("before load images and timestamps");
        LoadImagesAndTimestamps(startFrame, endFrame, path2imgs, imgFilenames, timestamps);

        int nImages = imgFilenames[0].size();

        MultiColSLAM::cSystem MultiSLAM(path2voc, path2settings, path2calibrations, true);

        // Vector for tracking time statistics
        vector<float> vTimesTrack;
        vTimesTrack.resize(nImages);

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;

        // Main loop
        const int nrCams = static_cast<int>(imgFilenames.size());
        std::vector<cv::Mat> imgs(nrCams);
        for (int ni = 0; ni < nImages; ni++)
        {
                // Read image from file
                std::vector<bool> loaded(nrCams);
                for (int c = 0; c < nrCams; ++c)
                {
                        cv::Mat color_img = cv::imread(imgFilenames[c][ni], CV_LOAD_IMAGE_COLOR);
                        cv::cvtColor(color_img, imgs[c], CV_BGR2GRAY);
                        if (imgs[c].empty())
                        {
                                cerr << endl << "Failed to load image at: " << imgFilenames[c][ni] << endl;
                                return 1;
                        }
                }
                double tframe = timestamps[ni];
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                // Pass the image to the SLAM system
                MultiSLAM.TrackMultiColSLAM(imgs, tframe);

                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

                double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

                vTimesTrack[ni] = ttrack;

                // Wait to load the next frame
                double T = 0;
                if (ni < nImages - 1)
                        T = timestamps[ni + 1] - tframe;
                else if (ni > 0)
                        T = tframe - timestamps[ni - 1];
                //std::this_thread::sleep_for(std::chrono::milliseconds(30));

                if (ttrack < T)
                        std::this_thread::sleep_for(std::chrono::milliseconds(
                                                static_cast<long>((T - ttrack))));
        }

	//p_thread[0].join();
	//p_thread[1].join();
        //pthread_join(p_thread[0], (void **) &status);
        //pthread_join(p_thread[1], (void **) &status);

        // Stop all threads
        MultiSLAM.Shutdown();

        // Tracking time statistics
        sort(vTimesTrack.begin(), vTimesTrack.end());
        float totaltime = 0;
        for (int ni = 0; ni<nImages; ni++)
        {
                totaltime += vTimesTrack[ni];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
        cout << "mean tracking time: " << totaltime / nImages << endl;

        // Save camera trajectory
        MultiSLAM.SaveMKFTrajectoryLAFIDA("MKFTrajectory.txt");

        free(recv_buffer[0]);
    	free(recv_buffer[1]);
	return 0;
}


void LoadImagesAndTimestamps(const int startFrame,
                const int endFrame,
                const string path2imgs,
                vector<vector<string>> &vstrImageFilenames,
                vector<double> &vTimestamps)
{
        vstrImageFilenames.resize(1);
        //ifstream fTimes;
        //string strPathTimeFile = path2imgs + "/images_and_timestamps.txt";

        //fTimes.open(strPathTimeFile.c_str());
        //string line;


        int cnt = 1;
        double timestamp = 0;
        //while (std::getline(fTimes, line))
        while (cnt <= endFrame)
        {
                //if (cnt >= startFrame && cnt < endFrame) // skip until startframe
                if (cnt >= startFrame)
                {
                        //std::istringstream iss(line);
                        //double timestamp;
                        //string pathimg1, pathimg2, pathimg3;
                        //if (!(iss >> timestamp >> pathimg1 >> pathimg2 >> pathimg3))
                        //      break;
                        timestamp += 40;
                        vTimestamps.push_back(timestamp);
			string pathimg1 = "../cam_img/img";
			string pathimg2 = to_string(cnt) + ".jpg";
                        vstrImageFilenames[0].push_back(pathimg1 + pathimg2);
                        //vstrImageFilenames[0].push_back(path2imgs + '/' + pathimg1);
                        //vstrImageFilenames[1].push_back(path2imgs + '/' + pathimg2);
                        //vstrImageFilenames[2].push_back(path2imgs + '/' + pathimg3);
                }
                ++cnt;

        }
}

/* thread functions */
//void* threadFcn0(void* data) // get packet and get 1 frame
void threadFcn0() // get packet and get 1 frame
{
        while (1){
                // receive packet from socket       
                recv_size = recvfrom(avb_socket, temp_buf, ETH_FRAME_LEN, 0, NULL, NULL);
                if (recv_size > 0) {
                        int eth_type = temp_buf[13]+((temp_buf[12]+1) << 8);
                        if( temp_buf[11] == 0x32 && eth_type == ETHERTYPE_AVTP ) {
                                recv_size -= AVTP_PAYLOAD_OFFSET;
                                nrPkt++;
                                memcpy(recv_buffer[nrContainer & 1] + rec_written[nrContainer & 1], temp_buf + AVTP_PAYLOAD_OFFSET, recv_size);
                                rec_written[nrContainer & 1] += recv_size;
                                if(nrPkt >= PACKET_LIMIT) {
                                        nrContainer++;
                                        nrPkt = 0;
                                        rec_written[nrContainer & 1] = 0;
                                }
                                if (nrVideo<nrContainer){
					gcond.notify_all();
                                        //pthread_cond_broadcast(&gcond);
                                }
                        }
                }
                if (nCapturedImage>1000) break;
        }
}

void threadFcn1()
{
        while (1)
        {
		std::unique_lock<std::mutex> lock(gmutex);
		gcond.wait(lock);
                //pthread_mutex_lock(&gmutex);
                //pthread_cond_wait(&gcond, &gmutex);
                if(nrVideo<nrContainer) {
                        int find;
                        //printf("Find the start of Frame: 0x00 0x00 0x00 0x01 0x67 0x42 \n");
                        for (find = 0; find < rec_written[nrVideo&1]; find++) {
                                if(recv_buffer[nrVideo&1][find]== 0x00) {
                                        if (recv_buffer[nrVideo&1][find + 5] == 0x42) {
                                                break;
                                        } else {}
                                } else {}
                        }
                        //printf("Save a capture file\n");
                        sprintf(path2video,"../cam_video/%d.h264",nrVideo);
                        f_vid = fopen(path2video,"wb");
                        fwrite(recv_buffer[nrVideo&1] + find, 1, rec_written[nrVideo&1] - find , f_vid);
                        VideoCapture cap1(path2video);
                        cap1.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H','2','6','4'));
                        if (!cap1.isOpened())
                        {
                                printf("동영상 파일을 열 수 없습니다. \n");
                        }
                        cap1.set(CAP_PROP_FRAME_WIDTH,1280);
                        cap1.set(CAP_PROP_FRAME_HEIGHT,720);

                        Mat frame1;
                        //namedWindow("video", 1);
                        for (;;){
                                //웹캡으로부터 한 프레임을 읽어옴  
                                cap1 >> frame1;
                                if (frame1.empty()){
                                        // reach to the end of the video file.
                                        break;
                                }
                                //imshow("video",frame1);
                                //if ( waitKey(20) == 27 ) break;
                                sprintf(imgDir, "../cam_img/img%d.jpg",nCapturedImage);
                                imwrite(imgDir,frame1);
                                nCapturedImage++;
                        }
                        nrVideo++;
                        fflush(f_vid);
                        fclose(f_vid);
			//printf ("%d\n",nCapturedImage);
                        if (nCapturedImage>1000) break;
                }
		lock.unlock();
                //pthread_mutex_unlock(&gmutex);
        }
}

int socketOpen()
{
        char if_name[IFNAMESIZ];
        strcpy(if_name, DEFAULT_IF);

        // open socket
        avb_socket = socket(AF_PACKET, SOCK_RAW,htons(ETH_P_ALL));
        if (avb_socket == -1) {
                printf("Fail to create a socket!\n");
                return -1;
        }
        printf(" open socket \n");

        // set recieve timeout
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        setsockopt( avb_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv) );
        // get socket buffer size
        int intSockbufSize = 256*1024;
        int intOptlen = sizeof(int);
        setsockopt(avb_socket, SOL_SOCKET, SO_SNDBUF, &intSockbufSize, (socklen_t)intOptlen);
        setsockopt(avb_socket, SOL_SOCKET, SO_RCVBUF, &intSockbufSize, (socklen_t)intOptlen);
	

        // set network card to promiscuos
        struct ifreq ethreq;
        strncpy(ethreq.ifr_name, "eth0", IFNAMSIZ);
        if (ioctl(avb_socket, SIOCGIFFLAGS, &ethreq) == -1) {
                perror("ioctl");
                printf("[VA_ERR] Network device search failed\n");
                close(avb_socket);
                return -1;
        }
        ethreq.ifr_flags |= IFF_PROMISC;
        if (ioctl(avb_socket, SIOCSIFFLAGS, &ethreq) == -1) {
                perror("ioctl");
                printf("[VA_ERR] Promiscuous mode setup failed\n");
                close(avb_socket);
                return -1;
        }
        // get ethernet index
        size_t if_name_len=strlen(if_name);
        if (if_name_len<sizeof(ifr.ifr_name)) {
                memcpy(ifr.ifr_name,if_name,if_name_len);
                ifr.ifr_name[if_name_len]=0;
        } else {
                printf("Error : interface name is too long\n");
        }
        if (ioctl(avb_socket,SIOCGIFINDEX,&ifr)==-1) {
                printf("Error : %s\n",strerror(errno));
        }

        // initialize socket address structure
        socketAddress.sll_ifindex = ifr.ifr_ifindex;
        socketAddress.sll_halen = ETH_ALEN;
        boolIsEnabled = true;
        return 1;
}
