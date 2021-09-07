#ifndef RECR_SCRIPT_H
#define RECR_SCRIPT_H
#include "action_manager.h"

class CRecreationalScript : public CActionManager
{
public:
	CRecreationalScript();
	~CRecreationalScript();
    void Queue();
};

#endif // RECR_SCRIPT_H