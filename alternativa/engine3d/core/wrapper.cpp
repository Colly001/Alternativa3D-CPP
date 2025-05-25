#include "wrapper.h"
#include "vertex.h"
#include "_global.h"
#include "wrapper.h"

std::shared_ptr<Wrapper> Wrapper::collector;




Wrapper::Wrapper()
{
	_function_name("Wrapper::Wrapper");
	this->_next = nullptr;
	this->vertex = nullptr;
}

Wrapper::~Wrapper()
{

}



void Wrapper::destroy(std::shared_ptr<Wrapper> &wrapper)
{
	_function_name("Wrapper::destroy");

	Wrapper *temp = wrapper.get();
	Wrapper *last = temp;

	while (temp != nullptr)
	{
		//Vertex *v = temp->vertex.get();
		//if (v != nullptr)
			//v->destroy();
		//temp->vertex = nullptr;
		last = temp;
		temp = temp->_next;
	}

	last->setNext(Wrapper::collector);
	Wrapper::collector = wrapper;

	/*
	while (this->next != nullptr)
	{
		//Vertex *v = this->next->vertex.get();
		//if (v != nullptr)
			//v->destroy();
		std::shared_ptr<Wrapper> temp = this->next;
		this->next = nullptr;
		this->next = temp->next;
	}

	if (this->vertex != nullptr)
	{
		this->vertex->destroy();
		this->vertex = nullptr;
	}
	*/
}

std::shared_ptr<Wrapper> Wrapper::create(int id)
{
	_function_name("Wrapper::create");

	if (collector != nullptr)
	{
		std::shared_ptr<Wrapper> wrapper = collector;
		collector = collector->next();
		wrapper->setNext(nullptr);
		wrapper->id = id;
		return wrapper;
	}

	auto p = std::make_shared<Wrapper>();
	p->id = id;
	return p;
}

