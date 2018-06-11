/*
* CommUtils.h
*
*  Created on: May 26, 2016
*      Author: laptopyellow
*/

#ifndef COMMUTILS_H_
#define COMMUTILS_H_

#include <string>
#include <exception>
using namespace std;

struct CommunicationException : public exception
{
	string message;
	CommunicationException(string exceptionMessage) : message(exceptionMessage) {}
	~CommunicationException() throw () {}

	const char* what() const throw () {
		return message.c_str();
	}
};



#endif /* COMMUTILS_H_ */
