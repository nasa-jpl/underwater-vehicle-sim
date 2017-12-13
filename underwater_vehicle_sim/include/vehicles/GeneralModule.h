#ifndef MODULE_H
#define MODULE_H

class GeneralModule
{
	public:
		GeneralModule(){}
		virtual ~GeneralModule(){}

		virtual void update()=0;
};


#endif