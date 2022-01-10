# Dijkstra in python

import numpy as np

#import math

def setupgraph(G,b,s):
	if s == 0:
		G[G==0] = b
	elif s == 1:
		G[G==b] = 0
		
	return G
		
def exchangenode(G,a,b):
	buffer = G[:,a]
	G[:,a] = G[:,b]
	G[:,b] = buffer
	
	buffer = G[a,:]
	G[a,:] = G[b,:]
	G[b,:] = buffer
	
	return G
	
def listdijkstra(L,W,s,d):
	index = W.shape[0]
	s = int(s)
	d = int(d)
	
	while index > 0:
		if W[1,d] == W[W.shape[0]-1,d]:
			L = np.append(L,s)
			index = 0
		else:
			index2 = W.shape[0]
			while index2 > 0:
				if W[index2-1,d] < W[index2-2,d]:
					L = np.append(L,W[index2-1,0])
					L = listdijkstra(L,W,s,W[index2-1,0])
					index2 = 0
				else:
					index2 = index2-1
				index = 0
				
	L.astype(int)
	return L


def dijkstra(A,s,d):
	A = A.astype(float)
	s = int(s)
	d = int(d)

	if s==d and s == 10000000:
		e = 0
		L = np.array(s)
	else:
		A = setupgraph(A,np.inf,0)
		
		if d == 0:
			d = s
		A = exchangenode(A,0,s)
		
		lengthA = A.shape[0]
		W = np.zeros([lengthA,lengthA])
		for i in range(1,lengthA):
			W[0,i] = i
			W[1,i] = A[0,i]
			
		D = np.zeros([lengthA,2])
		for i in range(lengthA):
			D[i,0] = A[0,i]
			D[i,1] = i
			
		D2 = D[1:D.shape[0],:]
		L = 1
		while L <= W.shape[0]-2:
			L = L+1
			#D2 = np.sort(D2,axis=0)
			D2 = D2[D2[:,0].argsort()]
			#D2 = np.roll(D2,1,axis=0)
			D2[np.abs(D2)>1e9] = 1e9
			D2 = D2.astype(int)
			k = D2[0,1]
			W[L,0] = k
			D2 = np.delete(D2,0,axis=0)
			for i in range(D2.shape[0]):
				if D[D2[i,1],0] > D[k,0]+A[k,D2[i,1]]:
					D[D2[i,1],0] = D[k,0]+A[k,D2[i,1]]
					D2[i,0] = D[D2[i,1],0]
			
			for i in range(1,A.shape[0]):
				W[L,i] = D[i,0]
				
		if d == s and s == 10000000:
			L = 0
		else:
			L = d
			
		e = W[W.shape[0]-1,d]
		L = listdijkstra(L,W,s,d)
		
		#L.astype(int)
	
	return e,L
	
	
if __name__=="__main__":
	G = np.array([[0,3,9,0],[1,0,0,7],[3,2,0,6],[0,1,0,0]])
	G1 = np.array([[0,3,9,0,0,0,0],[0,0,0,7,1,0,0],[0,2,0,7,0,0,0],[0,0,0,0,0,2,8],\
				[0,0,4,5,0,9,0],[0,0,0,0,0,0,4],[0,0,0,0,0,0,0]])
	s = 0
	d = 6
	(e,L) = dijkstra(G1,s,d)
	print("e=")
	print(e)
	print("L=")
	print(L)