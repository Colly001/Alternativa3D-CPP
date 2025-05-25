#include "constraint.h"

Constraint::Constraint()
{
	world = nullptr;
	satisfied = false;
}

Constraint::~Constraint()
{

}

void Constraint::preProcess(double dt)
{
	(void)dt;
}

void Constraint::apply(double dt)
{
	(void)dt;
}
