from random import random 
import matplotlib.pyplot as plt 
import numpy as np 

# colour representations
b = 0
g = 1
y = 2
o = 3
n = 4

cmap = [y, g, b, o, o, g, b, o, y, g, b]

states = list(range(2, 12))

# state model p(x_k+1 | x_k = X, u_k)
p_state = [[0.85, 0.10, 0.05],  # u = -1
           [0.05, 0.90, 0.05],  # u =  0
           [0.05, 0.10, 0.85]]  # u = +1
           # X-1    X     X+1

# measurement model p(z_k | x_k) - certainty matrix
p_measure = [[0.60, 0.20, 0.05, 0.05, 0.10],    # blue state
            [0.20, 0.60, 0.05, 0.05, 0.10],     # green state
            [0.05, 0.05, 0.65, 0.15, 0.10],     # yellow state
            [0.05, 0.05, 0.20, 0.60, 0.10]]     # orange state
            # b     g      y     o     n


# actions
x = []
u = [1,1,1,1,1,1,1,1,0,1,1,1,None]
z = [n, o, y, g, b, n, g, b, g, o, y, g, b]

# initial state is random (uniform)
# x.append(states[int(random()*4)])
x.append(np.ones(11) / 11)

# Loop
for k in range(11):
    print(z[k+1])
    # state prediction
    state_prediction = np.zeros(11)
    for i in range(11):     # each state_prediction entry, x_k+1; looping for each state in one step
        s = 0
        # x_k+1 is X-1
        s += p_state[u[k]+1][0] * x[k][(i+1)%11] 
        # x_k+1 is X (stays in the same state)
        s += p_state[u[k]+1][1] * x[k][i%11]
        # x_k+1 is X+1 (state is the next one)
        s += p_state[u[k]+1][2] * x[k][(i-1)%11]
        # put into state prediction
        state_prediction[i] = s 
 
    # state update
    state_update = np.zeros(11)
    for i in range(11): #for each x_k+1 (11 offices)
        s = 0   # initialize sum for the denominator
        for j in range(11): # sum for denominator for each state
            s += p_measure[cmap[j]][z[k+1]] * state_prediction[j]
        state_update[i] = p_measure[cmap[i]][z[k+1]] * state_prediction[i] / s

    x.append(state_update)   # save into the state all 11 office measurement probabilities
 
# display

fig, axes = plt.subplots(4, 3)
fig.tight_layout(pad=0.7)

colours = ['blue', 'green', 'gold', 'red']
bar_colours = [colours[i] for i in cmap]

print(bar_colours)

i = 0
for ax in axes.flat:
    ax.bar(range(2, 13), x[i], color=bar_colours)
    ax.set_xticks(np.arange(2, 13, step=1))
    ax.set_yticks(np.arange(0, 1, 0.25))
    ax.tick_params(axis='both', labelsize=5)
    ax.set_title(f"k={i+1}, u={u[i]}, z_k+1={z[i+1]}", size=5)
    i += 1

plt.show()

# fig, ax = plt.subplots(1, 1)
# ax.bar(range(2, 13), x[0], color=bar_colours)
# plt.show()