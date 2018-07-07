def factorial(n):
    f=1
    for i in range (1,n+1):
        f = i * f
        print f
    return f
import sys
if __name__ == '__main__':
    print factorial(int(sys.argv[1]))
