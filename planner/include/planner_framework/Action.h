#ifndef ACTION_H
#define ACTION_H

class Action
{
public:
	enum State {planned, executing, completed};

	Action() {}
	virtual ~Action() {}

	virtual void execute()=0;
	virtual bool tiggerReplan()=0;
	virtual void updateState()=0;

	void reset();
	State getState();
private:
	State state; 
};

#endif