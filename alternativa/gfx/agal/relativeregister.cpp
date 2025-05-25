#include "relativeregister.h"

RelativeRegister::RelativeRegister(int param1, int param2) :
	CommonRegister(param1, param2)
{

}

CommonRegister* RelativeRegister::rel(Register *param1, uint32_t param2)
{
	relate(param1, param2);
	return this;
}
