#ifndef WRAPPER_H
#define WRAPPER_H

#include <memory>

class Vertex;


class Wrapper : public std::enable_shared_from_this<Wrapper>
{
public:

	int id;

	std::shared_ptr<Wrapper> ptr()
	{
		return shared_from_this();
	}

	static std::shared_ptr<Wrapper> ptr_safe(Wrapper *t)
	{
		return t == nullptr ? nullptr : t->shared_from_this();
	}


	static std::shared_ptr<Wrapper> collector;

	Wrapper *_next;
	std::shared_ptr<Wrapper> p_next;

	std::shared_ptr<Wrapper> next()
	{
		return p_next;
	}

	inline void setNext(const std::shared_ptr<Wrapper> &next)
	{
		this->p_next = next;
		this->_next = next.get();
	}



	inline void setNext(const std::nullptr_t)
	{
		this->p_next = nullptr;
		this->_next = nullptr;
	}

	std::shared_ptr<Vertex> vertex;

	Wrapper();
	~Wrapper();

	static void destroy(std::shared_ptr<Wrapper> &wrapper);
	static std::shared_ptr<Wrapper> create(int id);
};


#endif // WRAPPER_H
