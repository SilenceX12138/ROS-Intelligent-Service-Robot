/* Copiright belong to xfyun(IFLYTEK CO.,LTD)                            */
/* @modified by Zhang Wanjie                                             */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include "xfyun_waterplus/IATSwitch.h" 

static bool bCue = true;	//true-语音识别前有提示音; false-语音识别前没有提示音
static std::string strTonePlay;

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

static ros::Publisher iat_pub;
static int nRecDuring = 10;

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;
void on_result(const char *result, char is_last)
{
	if (result) 
    {
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
        printf("\r识别结果:[ %s ]\n",  g_result);
        if(is_last) putchar('\n');

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

	printf("开始录音...\n");

	if(bCue == true)
	{
		std::stringstream ss;
		ss << strTonePlay; //16khz,单声道
		system(ss.str().c_str());
	}
}

void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("正在准备下一次录音,请稍等... \n");
	else
		printf("\nRecognizer error %d\n", reason);
}

static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	while(i++ < nRecDuring)
		sleep(1);
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}

static bool bActive = false;
bool iat_start(xfyun_waterplus::IATSwitch::Request  &req, xfyun_waterplus::IATSwitch::Response &res)
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

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "xfyun_iat_node");

    ros::NodeHandle n;
    iat_pub = n.advertise<std_msgs::String>("/xfyun/iat", 20);
    ros::ServiceServer start_svr = n.advertiseService("xfyun_waterplus/IATSwitch", iat_start);

	ros::NodeHandle n_param("~");
    bool bCN = false;
    n_param.param<bool>("cue", bCue, true);
    n_param.param<bool>("cn", bCN, false); 
    n_param.param<bool>("start", bActive, true);

	char const* home_dir = getenv("HOME");
	std::string strHomeDir = home_dir;
	strTonePlay = "aplay -q " + strHomeDir + "/catkin_ws/src/xfyun_waterplus/sound/on.wav";

	int ret = MSP_SUCCESS;
	const char* login_params = "appid = 58eeeedf, work_dir = .";

	char session_begin_params[512];
	if(bCN == true)
		strcpy(session_begin_params,"sub = iat, ptt = 0, domain = iat, language = zh_cn, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8");
	else
		strcpy(session_begin_params,"sub = iat, ptt = 0, domain = iat, language = en_us, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8");

	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		MSPLogout();
		return 0;
	}

    ros::Rate r(10);

	int rou = 0;

	while(n.ok())
	{
		// printf("A round %d", rou++);
		if(bActive == true)
		{
			printf("请在 %d 秒内下达命令\n",nRecDuring);
			demo_mic(session_begin_params);
			printf("开始识别\n");
		}
		// printf("B round %d", rou++);
        ros::spinOnce();
		// printf("C round %d", rou++);
		r.sleep();
	} 

	MSPLogout(); 

	return 0;
}