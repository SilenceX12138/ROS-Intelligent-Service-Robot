/* Copiright belong to xfyun(IFLYTEK CO.,LTD)                            */
/* @modified by Zhang Wanjie                                             */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <iconv.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

static ros::Publisher iat_pub;
static char session_begin_params[512];
static bool bCN = false;
static bool bActive = false;

static void show_result(char *string, char is_over)
{
	if(bCN == false)
	{
		ROS_WARN("[%s]",string);
	}
	else
	{
		printf("\r识别结果:[ %s ]\n",  string);
	}

    if(is_over)
		putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);

		std_msgs::String strResult;
		std::string str(result);
		strResult.data = str;
		iat_pub.publish(strResult);
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	//printf("Start Listening...\n");
	printf("开始语音识别...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		//printf("\nSpeaking done \n");
		printf("正在准备下一次语音识别,请稍等... \n");
	else
		printf("\nRecognizer error %d\n", reason);
}

/* demo send audio data from a file */
static void demo_file(const char* audio_file, const char* session_begin_params)
{
	int	errcode = 0;
	FILE*	f_pcm = NULL;
	char*	p_pcm = NULL;
	unsigned long	pcm_count = 0;
	unsigned long	pcm_size = 0;
	unsigned long	read_size = 0;
	struct speech_rec iat;
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	if (NULL == audio_file)
		goto iat_exit;

	f_pcm = fopen(audio_file, "rb");
	if (NULL == f_pcm)
	{
		printf("\nopen [%s] failed! \n", audio_file);
		goto iat_exit;
	}

	fseek(f_pcm, 0, SEEK_END);
	pcm_size = ftell(f_pcm);
	fseek(f_pcm, 0, SEEK_SET);

	p_pcm = (char *)malloc(pcm_size);
	if (NULL == p_pcm)
	{
		printf("\nout of memory! \n");
		goto iat_exit;
	}

	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
	if (read_size != pcm_size)
	{
		printf("\nread [%s] error!\n", audio_file);
		goto iat_exit;
	}

	errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed : %d\n", errcode);
		goto iat_exit;
	}

	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("\nsr_start_listening failed! error code:%d\n", errcode);
		goto iat_exit;
	}

	while (1)
	{
		unsigned int len = 10 * FRAME_LEN; /* 200ms audio */
		int ret = 0;

		if (pcm_size < 2 * len)
			len = pcm_size;
		if (len <= 0)
			break;

		ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);

		if (0 != ret)
		{
			printf("\nwrite audio data failed! error code:%d\n", ret);
			goto iat_exit;
		}

		pcm_count += (long)len;
		pcm_size -= (long)len;		
	}

	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("\nsr_stop_listening failed! error code:%d \n", errcode);
		goto iat_exit;
	}

iat_exit:
	if (NULL != f_pcm)
	{
		fclose(f_pcm);
		f_pcm = NULL;
	}
	if (NULL != p_pcm)
	{
		free(p_pcm);
		p_pcm = NULL;
	}

	sr_stop_listening(&iat);
	sr_uninit(&iat);
}

void rec_file_callback(const std_msgs::String::ConstPtr& msg)
{
    const char* audio_file = msg->data.c_str();
	
	demo_file(audio_file,session_begin_params);
}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "xfyun_iat_record");

    ros::NodeHandle n;
    iat_pub = n.advertise<std_msgs::String>("/xfyun/iat", 10);
	ros::NodeHandle n_param("~");
    n_param.param<bool>("cn", bCN, false); 
    n_param.param<bool>("start", bActive, true);

	if(bCN == true)
		strcpy(session_begin_params,"sub = iat, ptt = 0, domain = iat, language = zh_cn, \
		accent = mandarin, sample_rate = 16000, \
		result_type = plain, result_encoding = utf8");
	else
		strcpy(session_begin_params,"sub = iat, ptt = 0, domain = iat, language = en_us, \
		accent = mandarin, sample_rate = 16000, \
		result_type = plain, result_encoding = utf8");

	int ret = MSP_SUCCESS;
	const char* login_params = "appid = 58eeeedf, work_dir = .";
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		MSPLogout();
		return 0;
	}

	ros::Subscriber sub =n.subscribe("/xfyun/reco_file",2,rec_file_callback);
	//sleep(1);
	//demo_file("/home/robot/s0.wav",session_begin_params);	//test
	ros::spin();

	MSPLogout(); // Logout...

	return 0;
}
