/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <alsa/asoundlib.h>
#include <string.h>
#include <signal.h>
#include <sound_play/SoundRequest.h>
#include "xfyun_waterplus/IATSwitch.h"

static ros::Publisher play_pub;
static int nRecDuring = 10;

#define BUFSIZE		(1024 * 4)

FILE *capture_file;
unsigned int size = 0;
snd_pcm_t *capture_handle;

void write_wav_header(unsigned char *header, uint16_t wFormatTag, uint16_t wChannels, 
			uint32_t dwSamplesPerSec, uint16_t wBitsPerSample)
{
	uint32_t dwAvgBytesPerSec;
	uint16_t wBlockAlign;

	/* writing riff */
	unsigned char riff[12] = { 'R', 'I', 'F', 'F',		/* RIFF flag */ 
	                     0 ,  0 ,  0 ,  0 ,		/* File size */
	                    'W', 'A', 'V', 'E' };	/* WAVE flag*/

	memcpy(&header[0], &riff[0], sizeof(riff));

	/* writing fmt */
	unsigned char fmt[24] = {  'f',  'm',  't',  ' ',	/* fmt flag */
	                   0x10, 0x00, 0x00, 0x00 };	/* fmt length */
	
	
	/* format, channels */
	fmt[8] = wFormatTag & 0xFF;
	fmt[9] = (wFormatTag >> 8) & 0xFF;
	fmt[10] = wChannels & 0xFF;
	fmt[11] = (wChannels >> 8) & 0xFF;

	/* sample rate */
	fmt[12] = dwSamplesPerSec & 0xFF;
	fmt[13] = (dwSamplesPerSec >> 8) & 0xFF;
	fmt[14] = (dwSamplesPerSec >> 16) & 0xFF;
	fmt[15] = (dwSamplesPerSec >> 24) & 0xFF;
	
	/* frame size */
	if (wFormatTag) 
		wBlockAlign = wChannels * ((wBitsPerSample + 7) / 8);
	else 
		wBlockAlign = wBitsPerSample / 8 * wChannels;

	fmt[20] = wBlockAlign & 0xFF;
	fmt[21] = (wBlockAlign >> 8) & 0xFF;
	fmt[22] = wBitsPerSample & 0xFF;
	fmt[23] = (wBitsPerSample >> 8) & 0xFF;
	
	/* bytes per second */
	dwAvgBytesPerSec = dwSamplesPerSec * wBlockAlign;
	fmt[16] = dwAvgBytesPerSec & 0xFF;
	fmt[17] = (dwAvgBytesPerSec >> 8) & 0xFF;
	fmt[18] = (dwAvgBytesPerSec >> 16) & 0xFF;
	fmt[19] = (dwAvgBytesPerSec >> 24) & 0xFF;
	

	memcpy(&header[12], &fmt[0], sizeof(fmt));
	
	/* writing data (header) */
	unsigned char data[8] = {  'd',  'a',  't',  'a',
	                   0x00, 0x00, 0x00, 0x00 };	/* data chunk size */

	memcpy(&header[36], &data[0], sizeof(data));
}

void update_wav_size(unsigned char *file, int file_len)
{
	file[7] = (file_len >> 24) & 0xFF;
	file[6] = (file_len >> 16) & 0xFF;
	file[5] = (file_len >> 8 ) & 0xFF,
	file[4] =  file_len & 0xFF; 

	int data_len = file_len - 44;
	file[43] = (data_len >> 24) & 0xFF;
	file[42] = (data_len >> 16) & 0xFF;
	file[41] = (data_len >> 8 ) & 0xFF,
	file[40] =  data_len & 0xFF; 
}

void file_update_wav_size()
{
	unsigned char buf[4];
	buf[3] = (size >> 24) & 0xFF;
	buf[2] = (size >> 16) & 0xFF;
	buf[1] = (size >> 8 ) & 0xFF,
	buf[0] =  size & 0xFF; 

	fseek(capture_file, 4, SEEK_SET);
	fwrite(&buf, sizeof(buf), 1, capture_file);

	int data_len = size - 44;
	buf[3] = (data_len >> 24) & 0xFF;
	buf[2] = (data_len >> 16) & 0xFF;
	buf[1] = (data_len >> 8 ) & 0xFF,
	buf[0] =  data_len & 0xFF; 

	fseek(capture_file, 40, SEEK_SET);
	fwrite(&buf, sizeof(buf), 1, capture_file);
}

void stop(int sig) {
	file_update_wav_size();
	fclose(capture_file);
	//snd_pcm_close (capture_handle);

	fprintf(stdout, "exited...\n");
	exit(0);
}

static bool bActive = false;
bool sr_switch(xfyun_waterplus::IATSwitch::Request  &req, xfyun_waterplus::IATSwitch::Response &res)
{
	bActive = req.active;
	if(bActive == true)
	{
		nRecDuring = req.duration;
		printf("[IATSwitch] 语音识别开启! 识别时长为 %d 秒\n",nRecDuring);
	}
	else
	{
		printf("[IATSwitch] 语音识别关闭!\n");
	}
	return true;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "waterplus_recorder_wav");

	char const* home_dir = getenv("HOME");
	std::string strHomeDir = home_dir;
	std::string strTonePlay = strHomeDir + "/catkin_ws/src/xfyun_kinetic/sound/on.wav";

	int ret;
	short buf[BUFSIZE * 2];

	//char device[] = "hw:1,0";
	char device[] = "default";
	snd_pcm_format_t fmt = SND_PCM_FORMAT_S16_LE;
	//snd_pcm_format_t fmt = SND_PCM_FORMAT_U8;
	int rate = 44100;
	int channels = 2;
	int bits = 16;

	char file_name[] = "/dev/shm/out.wav";

	int c;
	extern char *optarg;
	extern int optopt, optind;
	int f_set = 0;
	int r_set = 0;
	int d_set = 0;

	unsigned char wav_header[44];

	if ((ret = snd_pcm_open (&capture_handle, device, SND_PCM_STREAM_CAPTURE, 0)) < 0) 
	{
		printf ("Cannot open audio device %s (%s)\n", device,snd_strerror(ret));
		exit(1);
	} 

	if ((ret = snd_pcm_set_params(capture_handle, fmt,
					SND_PCM_ACCESS_RW_INTERLEAVED,
					channels,
					rate,
					1,
					500000)) < 0 ) 
	{
		printf("Playback open error (%s)\n", snd_strerror(ret));
		snd_pcm_close (capture_handle);
		exit(1);
	}

	if ((ret = snd_pcm_prepare (capture_handle)) < 0) 
	{
		fprintf(stderr, "Cannot prepare pcm for use (%s)\n", 
			snd_strerror(ret));
		snd_pcm_close (capture_handle);
		exit(1);
	}

	if ((ret = snd_pcm_start (capture_handle)) < 0) 
	{
		fprintf(stderr, "Cannot prepare pcm for use (%s)\n", 
			snd_strerror(ret));
		snd_pcm_close (capture_handle);
		exit(1);
	}

	ros::NodeHandle n;
	ros::Publisher snd_pub = n.advertise<std_msgs::String>("/xfyun/snd_file", 10);
    play_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    ros::ServiceServer switch_svr = n.advertiseService("xfyun_waterplus/IATSwitch", sr_switch);
	ros::NodeHandle n_param("~");
    n_param.param<bool>("start", bActive, true);

	ros::Rate loop_rate(10);

	sleep(1);
	int count = 0;
	while (ros::ok())
	{
		if(bActive == true)
		{
			ROS_INFO("new record");
			std_msgs::String msg;
			std::stringstream ss;
			//ss << "/home/robot/speech_" << count << ".wav";
			ss << "/dev/shm/speech_" << count << ".wav";
			
			memset(buf, 0, sizeof(buf));

			capture_file = fopen(ss.str().c_str(), "w+");

			if (capture_file == NULL) {
				fprintf(stderr, "Cannot open file %s\n", file_name);
				snd_pcm_close (capture_handle);
				exit(1);
			}

			write_wav_header(wav_header, 1, channels, rate, bits);
			fwrite(&wav_header, sizeof(wav_header), 1, capture_file);

			signal(SIGINT, stop);

			//提示音
			sound_play::SoundRequest sp;
			sp.sound = sound_play::SoundRequest::PLAY_FILE;
			sp.command = sound_play::SoundRequest::PLAY_ONCE;
			sp.arg = strTonePlay;
			play_pub.publish(sp);
			usleep(1000*1000);

			while(1) 
			{
				if(bActive == false)
					break;
				int avail = snd_pcm_avail_update (capture_handle);
				if (avail > 0) 
				{
					if (avail > BUFSIZE);
						avail = BUFSIZE;
					snd_pcm_readi (capture_handle, buf, avail);
					fwrite(&buf, sizeof(buf), 1, capture_file);
					size += sizeof(buf);
					ROS_INFO("size = %d ",size);
					if(size >= nRecDuring*1024*1024/6)		//录音时长
						break;
				}
				else
				{
					printf("音频数据流错误: avail = %d \n",avail);
				}
			}
			if(bActive == false)
			{
				size = 0;
				continue;
			}

			file_update_wav_size();
			fclose(capture_file);
			size = 0;
			
			msg.data = ss.str();
			ROS_INFO("%s", msg.data.c_str());
			snd_pub.publish(msg);
			ros::spinOnce();

			loop_rate.sleep();
			++count;
			if(count > 10)
				count = 0;
		}
		else
		{
			//将数据读出来并抛弃
			int avail = snd_pcm_avail_update (capture_handle);
			if (avail > 0) 
			{
				if (avail > BUFSIZE);
						avail = BUFSIZE;
			 	snd_pcm_readi (capture_handle, buf, avail);
			}
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	snd_pcm_close (capture_handle);

	return 0;
}