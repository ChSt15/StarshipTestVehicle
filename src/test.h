
class Thread {
public:

    virtual void loop() = 0;

};


class Scheduler {
public:

    void addItem(Thread* threadPtr) {
        //Place pointer to thread in a list to run when runItems() is called.
    }

    void runItems();

};


class Task_Abstract: public Thread {
public:

    Task_Abstract() {
        scheduler.addItem(this);
    }

    static Scheduler scheduler;

};



//Following is in .cpp file that includes the header file.

Scheduler Task_Abstract::scheduler;


