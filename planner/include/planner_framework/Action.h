#ifndef ACTION_H
#define ACTION_H

#include <memory>

class Action
{
public:
	enum State {PLANNED, EXECUTING, INTERRUPTED, COMPLETED, FAILED};

	Action();
	Action(const Action& action);
	virtual ~Action() {}

	virtual std::unique_ptr<Action> clone() const = 0;

	virtual void executeAction()=0;
	void execute();
	virtual bool triggerReplan()=0;
	virtual void monitor()=0;


	void reset();
	State getState();
	
protected:
	State state; 
};

#endif