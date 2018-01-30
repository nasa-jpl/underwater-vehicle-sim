#ifndef ACTION_H
#define ACTION_H

class Action
{
public:
	enum State {PLANNED, EXECUTING, INTERRUPTED, COMPLETED, FAILED};

	Action() {}
	virtual ~Action() {}

	virtual void execute()=0;
	virtual bool tiggerReplan()=0;
	virtual void monitor()=0;

	void reset();
	State getState();
	
protected:
	State state; 
};

#endif