#ifndef HOME_SCRIPT_H
#define HOME_SCRIPT_H
#include "action_manager.h"

class CHomeScript : public CActionManager
{
public:
	CHomeScript();
	~CHomeScript();
    void Queue();
};

#endif // HOME_SCRIPT_H