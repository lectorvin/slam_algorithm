middle = lambda arr: sum(arr)/len(arr)
mul = lambda arr1,arr2: [arr1[i]*arr2[i] for i in range(len(arr1))]
cov = lambda arr1,arr2: middle(mul(arr1,arr2)) - middle(arr1)*middle(arr2)

x = [[0,0,0],[0,1,1],[1,2,3],[1,3,3],[2,4,5]]
y = [[3,3],[3,4],[4,5],[4,6],[5,8]]

cov_matrix = lambda x,y: [[round(cov([k[i] for k in x],[t[j] for t in y]),2) for j in range(len(y[0]))] for i in range(len(x[0]))]
print(cov_matrix(y,y))
