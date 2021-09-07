#ifndef CAMP_SCRIPT_H
#define CAMP_SCRIPT_H
#include "action_manager.h"

class CCampusScript : public CActionManager
{
public:
	CCampusScript();
	~CCampusScript();
    void Queue();
};

#endif // CAMP_SCRIPT_H