#include "vertex.h"
#include "_global.h"


std::shared_ptr<Vertex> Vertex::collector;

static std::shared_ptr<Vertex> new_Vertex()
{
	return std::make_shared<Vertex>();
}


Vertex::Vertex()
{
	_function_name("Vertex::Vertex");
	x = y = z = u = v = 0.;
	offset = 1;
	transformId = 0;
	next = nullptr;
	value = nullptr;
	drawId = 0;
}

void Vertex::destroy()
{
	//_function_name("Vertex::destroy");

	while (this->next != nullptr)
	{
		std::shared_ptr<Vertex> temp = this->next;
		this->next = nullptr;
		this->next = temp->next;
	}

	while (this->value != nullptr)
	{
		std::shared_ptr<Vertex> temp = this->value;
		this->value = nullptr;
		this->value = temp->value;
	}
}



std::shared_ptr<Vertex> Vertex::createList(int count)
{
	std::shared_ptr<Vertex> vertex = collector;
	if (vertex != nullptr)
	{
		std::shared_ptr<Vertex> temp = vertex;

		while (count > 1)
		{
			temp->transformId = 0;
			temp->drawId = 0;
			if (temp->next == nullptr)
			{
				while (count > 1)
				{
					temp->next = new_Vertex();
					temp = temp->next;
					count--;
				}
				break;
			}
			temp = temp->next;
			count--;
		}

		collector = temp->next;
		temp->transformId = 0;
		temp->drawId = 0;
		temp->next = nullptr;
	}
	else
	{
		vertex = new_Vertex();

		std::shared_ptr<Vertex> temp = vertex;
		while (count > 1)
		{
			temp->next = new_Vertex();
			temp = temp->next;
			count--;
		}
	}

	return vertex;
}



std::shared_ptr<Vertex> Vertex::create()
{
	if (collector != nullptr)
	{
		std::shared_ptr<Vertex> vertex = collector;
		collector = vertex->next;
		vertex->next = nullptr;
		vertex->transformId = 0;
		vertex->drawId = 0;
		return vertex;
	}

	return new_Vertex();
}

