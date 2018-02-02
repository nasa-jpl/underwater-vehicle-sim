#ifndef ACTION_H
#define ACTION_H

#include <memory>

class Action
{
public:
	enum State {PLANNED, DISPATCHED, EXECUTING, INTERRUPTED, COMPLETED, FAILED};

	Action();
	Action(const Action& action);
	virtual ~Action() {}

	/**
	* Clone in order to deep copy the action
	*/
	virtual std::shared_ptr<Action> clone() const = 0;

	/**
	* Executes this action
	*/
	virtual void executeAction()=0;

	/**
	* Sets the state of the action before executing it
	*/
	void execute();

	/**
	* Determines if this action wants to trigger a replan. Returns true if a replan is desired
	*/
	virtual bool triggerReplan()=0;

	/**
	* Monitors the state of this action and updates the state as needed
	*/
	virtual void monitor()=0;

	/**
	*Resets this action to a state as if it has not been executed.
	*/
	void reset();

	/**
	* Gets the state of this action.
	*/
	State getState();

	void setState(State newState);
	
protected:
	State state; 
};

#endif