#ifndef MODULE_H
#define MODULE_H

class Module
{
	public:
		Module(){}
		virtual ~Module(){}

		virtual void simCycle()=0;
};


#endif