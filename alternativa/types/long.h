#ifndef LONG_H
#define LONG_H


class Long
{
	int _low;
	int _high;
public:

	Long(int low, int high)
	{
		this->_low = low;
		this->_high = high;
	}

	int low()
	{
		return this->_low;
	}

	int high()
	{
		return this->_high;
	}
};

#endif // LONG_H
