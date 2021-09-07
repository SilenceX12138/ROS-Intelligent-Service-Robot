#ifndef INNO_SCRIPT_H
#define INNO_SCRIPT_H
#include "action_manager.h"

class CInnovationScript : public CActionManager
{
public:
	CInnovationScript();
	~CInnovationScript();
    void Queue();
};

#endif // INNO_SCRIPT_H