def divisors(n):
    for i in range(1,int((n**.5)+1)):
        if n % i == 0 :
            print i, n/i
    
print divisors (72)
