/* Copiright belong to xfyun(IFLYTEK CO.,LTD)                            */
/* @modified by Zhang Wanjie                                             */
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#define	BUFFER_SIZE 2048
#define HINTS_SIZE  100
#define GRAMID_LEN	128
#define FRAME_LEN	640 

static ros::Publisher asr_pub;
static char*		grammar_id		=	NULL;

int get_grammar_id(std::string grammar_file, char* grammar_id, unsigned int id_len)
{
	FILE*			fp				=	NULL;
	char*			grammar			=	NULL;
	unsigned int	grammar_len		=	0;
	unsigned int	read_len		=	0;
	const char*		ret_id			=	NULL;
	unsigned int	ret_id_len		=	0;
	int				ret				=	-1;	

	if (NULL == grammar_id)
		goto grammar_exit;

	fp = fopen(grammar_file.c_str(), "rb");
	if (NULL == fp)
	{   
		printf("\nopen grammar file failed!\n");
		goto grammar_exit;
	}
	
	fseek(fp, 0, SEEK_END);
	grammar_len = ftell(fp); //获取语法文件大小 
	fseek(fp, 0, SEEK_SET); 

	grammar = (char*)malloc(grammar_len + 1);
	if (NULL == grammar)
	{
		printf("\nout of memory!\n");
		goto grammar_exit;
	}

	read_len = fread((void *)grammar, 1, grammar_len, fp); //读取语法内容
	if (read_len != grammar_len)
	{
		printf("\nread grammar error!\n");
		goto grammar_exit;
	}
	grammar[grammar_len] = '\0';

	ret_id = MSPUploadData("usergram", grammar, grammar_len, "dtt = abnf, sub = asr", &ret); //上传语法
	if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed, error code: %d.\n", ret);
		goto grammar_exit;
	}

	ret_id_len = strlen(ret_id);
	if (ret_id_len >= id_len)
	{
		printf("\nno enough buffer for grammar_id!\n");
		goto grammar_exit;
	}
	strncpy(grammar_id, ret_id, ret_id_len);
	printf("grammar_id: \"%s\" \n", grammar_id); //下次可以直接使用该ID，不必重复上传语法。

grammar_exit:
	if (NULL != fp)
	{
		fclose(fp);
		fp = NULL;
	}
	if (NULL!= grammar)
	{
		free(grammar);
		grammar = NULL;
	}
	return ret;
}

void run_asr(const char* audio_file, const char* params, char* grammar_id)
{
	const char*		session_id						= NULL;
	char			rec_result[BUFFER_SIZE]		 	= {'\0'};	
	char			hints[HINTS_SIZE]				= {'\0'}; //hints为结束本次会话的原因描述，由用户自定义
	unsigned int	total_len						= 0;
	int 			aud_stat 						= MSP_AUDIO_SAMPLE_CONTINUE;		//音频状态
	int 			ep_stat 						= MSP_EP_LOOKING_FOR_SPEECH;		//端点检测
	int 			rec_stat 						= MSP_REC_STATUS_SUCCESS;			//识别状态	
	int 			errcode 						= MSP_SUCCESS;

	FILE*			f_pcm 							= NULL;
	char*			p_pcm 							= NULL;
	long 			pcm_count 						= 0;
	long 			pcm_size 						= 0;
	long			read_size						= 0;

	if (NULL == audio_file)
		goto asr_exit;

	f_pcm = fopen(audio_file, "rb");
	if (NULL == f_pcm) 
	{
		printf("\nopen [%s] failed!\n", audio_file);
		goto asr_exit;
	}
	
	fseek(f_pcm, 0, SEEK_END);
	pcm_size = ftell(f_pcm); //获取音频文件大小 
	fseek(f_pcm, 0, SEEK_SET);		

	p_pcm = (char*)malloc(pcm_size);
	if (NULL == p_pcm)
	{
		printf("\nout of memory!\n");
		goto asr_exit;
	}

	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm); //读取音频文件内容
	if (read_size != pcm_size)
	{
		printf("\nread [%s] failed!\n", audio_file);
		goto asr_exit;
	}
	
	printf("\n开始语音识别 ...\n");
	session_id = QISRSessionBegin(grammar_id, params, &errcode);
	if (MSP_SUCCESS != errcode)
	{
		printf("\nQISRSessionBegin failed, error code:%d\n", errcode);
		goto asr_exit;
	}
	
	while (1) 
	{
		unsigned int len = 10 * FRAME_LEN; // 每次写入200ms音频(16k，16bit)：1帧音频20ms，10帧=200ms。16k采样率的16位音频，一帧的大小为640Byte
		int ret = 0;

		if (pcm_size < 2 * len) 
			len = pcm_size;
		if (len <= 0)
			break;
		
		aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
		if (0 == pcm_count)
			aud_stat = MSP_AUDIO_SAMPLE_FIRST;
		
		printf(">");
		ret = QISRAudioWrite(session_id, (const void *)&p_pcm[pcm_count], len, aud_stat, &ep_stat, &rec_stat);
		if (MSP_SUCCESS != ret)
		{
			printf("\nQISRAudioWrite failed, error code:%d\n",ret);
			goto asr_exit;
		}
			
		pcm_count += (long)len;
		pcm_size  -= (long)len;
		
		if (MSP_EP_AFTER_SPEECH == ep_stat)
			break;
		usleep(200*1000); //模拟人说话时间间隙，10帧的音频长度为200ms
	}
	errcode = QISRAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST, &ep_stat, &rec_stat);
	if (MSP_SUCCESS != errcode)
	{
		printf("\nQISRAudioWrite failed, error code:%d\n",errcode);
		goto asr_exit;	
	}

	while (MSP_REC_STATUS_COMPLETE != rec_stat) 
	{
		const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
		if (MSP_SUCCESS != errcode)
		{
			printf("\nQISRGetResult failed, error code: %d\n", errcode);
			goto asr_exit;
		}
		if (NULL != rslt)
		{
			unsigned int rslt_len = strlen(rslt);
			total_len += rslt_len;
			if (total_len >= BUFFER_SIZE)
			{
				printf("\nno enough buffer for rec_result !\n");
				goto asr_exit;
			}
			strncat(rec_result, rslt, rslt_len);
		}
		usleep(150*1000); //防止频繁占用CPU
	}
	printf("\n语音识别结束\n");
	printf("=============================================================\n");
	printf("%s",rec_result);
	printf("=============================================================\n");

asr_exit:
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

	QISRSessionEnd(session_id, hints);
}

static const char*	session_begin_params	=	"sub = asr, result_type = plain, result_encoding = utf8,sample_rate = 16000,aue = speex-wb";
void rec_file_callback(const std_msgs::String::ConstPtr& msg)
{
    const char* audio_file = msg->data.c_str();
	//ROS_INFO("[rec_file ] %s",audio_file);
    
	run_asr(audio_file, session_begin_params, grammar_id);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "xfyun_asr_node");
    ros::NodeHandle n;
    asr_pub = n.advertise<std_msgs::String>("/xfyun/asr", 20);
	ros::Subscriber sub =n.subscribe("/xfyun/reco_file",2,rec_file_callback);

	char const* home_dir = getenv("HOME");
	std::string strHomeDir = home_dir;
	std::string strFile = strHomeDir + "/catkin_ws/src/wpb_home_apps_2017/asr/1_follow.bnf";

    int			ret						=	MSP_SUCCESS;
	const char* login_params			=	"appid = 58eeeedf, work_dir = .";
    ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，均传NULL即可，第三个参数是登录参数
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n",ret);
		goto exit; //登录失败，退出登录
	}


    grammar_id = (char*)malloc(GRAMID_LEN);
	if (NULL == grammar_id)
	{
		printf("out of memory !\n");
		goto exit;
	}
	memset(grammar_id, 0, GRAMID_LEN);

	printf("上传语法 ...\n");
	ret = get_grammar_id(strFile, grammar_id, GRAMID_LEN);
	if (MSP_SUCCESS != ret)
		goto exit;
	printf("上传语法成功\n");

    ros::spin();

exit:
	if (NULL != grammar_id)
	{
		free(grammar_id);
		grammar_id = NULL;
	}
	MSPLogout(); //退出登录
}