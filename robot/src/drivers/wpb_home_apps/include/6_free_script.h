#ifndef FREE_SCRIPT_H
#define FREE_SCRIPT_H
#include "action_manager.h"

class CFreeScript : public CActionManager
{
public:
	CFreeScript();
	~CFreeScript();
    void Queue();
};

#endif // FREE_SCRIPT_H