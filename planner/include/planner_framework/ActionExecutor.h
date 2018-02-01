#ifndef ACTION_EXECUTOR_H
#define ACTION_EXECUTOR_H

template <class T>
class ActionExecutor
{
public:
	ActionExecutor() {};
	virtual ~ActionExecutor() {}

	virtual bool execute(T& action)=0;
	
	/**
	* Monitors and updates the state of the yoyo action in the ros simulation 
	*/
	virtual void monitor(T& action)=0;

	/**
	* Allows the yoyo action to trigger a replan in the ros simulation 
	*/
	virtual bool triggerReplan(T& action)=0;

private:

};
#endif