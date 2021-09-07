#ifndef INNO_SCRIPT_H
#define INNO_SCRIPT_H
#include "action_manager.h"

class FwhScript : public CActionManager
{
public:
	FwhScript();
	~FwhScript();
    void Queue();
};

#endif // INNO_SCRIPT_H
