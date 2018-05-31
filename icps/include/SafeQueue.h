/*
 * SafeQueue.h
 *
 *  Created on: Oct 8, 2017
 *      Author: retry
 */

#ifndef SAFEQUEUE_H_
#define SAFEQUEUE_H_
#include <mutex>
#include <condition_variable>
#include <queue>

using namespace std;

template <class T>
class SafeQueue {
private:
    queue<T> q;
    mutable mutex m;
    condition_variable c;
    bool hasWait;
public:
	SafeQueue(): q(), m(), c(), hasWait(false){}
	virtual ~SafeQueue(){}

	void enqueue(T _msg){
		std::lock_guard<std::mutex> lock(m);	//lock_guard lock mutex given to it until it is destroy (out of current scope)
		q.push(_msg);
		c.notify_one();
	}

	T dequeue(){	//Only one dequeue call at a time
		std::unique_lock<std::mutex> lock(m);
	    while(q.empty())
	    {
	    	hasWait = true;
	    	c.wait(lock);	// temporary release lock as long as condition_variable is not notified and reaquire it afterwards.
	    }
	    T val = q.front();
	    q.pop();
	    hasWait = false;
	    return val;
	}

	bool getHasWait(){return hasWait;}
};

#endif /* SAFEQUEUE_H_ */
