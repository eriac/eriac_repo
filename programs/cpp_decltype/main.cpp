#include <stdio.h>
#include <memory>

struct A{
	std::shared_ptr<int> a{std::make_shared<int>()};
};

struct B {
	A a;
	int b;
};

struct Target {
	int t;
};

struct Command {
	Target target;
};

template<class T> struct Accesstor {
	decltype(T::target) target;
};

int main(void) {
	Accesstor<Command> my_command;
	my_command.target.t = 1;
	printf("aaaaa %d\n", my_command.target.t);
}
