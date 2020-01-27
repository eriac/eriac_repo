import cvxpy
import numpy

m = 10
n = 5
numpy.random.seed(1)
A = numpy.array([[1.5544,2],[3.343,4.456]])
b = numpy.array([1,2])

x = cvxpy.Variable(2)
objective = cvxpy.Minimize(cvxpy.sum_squares(A * x - b))
constraints = [0 <= x, x <= 1]
prob = cvxpy.Problem(objective, constraints)

result = prob.solve()
print("optimal parameter:\n", x.value)
print("Lagrange parameter\n", constraints[0].dual_value)
print("status:" + prob.status)

print(A)
print(numpy.dot(A,x.value))
print(b)
print(numpy.dot(A,x.value)-b)

print("math")
c=numpy.dot(numpy.linalg.inv(A),b)
print(c)
print(numpy.dot(A,c)-b)


