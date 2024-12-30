import numpy as np
import pickle

# var = 0.04
# # 20 random samples linearly spaced between -var and var
# samples = np.random.uniform(-var, var, (20, 2))
# for i in range(20):
#     dx = samples[i, 0]
#     dy = samples[i, 1]
#     print(f"dx: {dx}, dy: {dy}")

# # save the offsets
# with open("offsets.pkl", "wb") as f:
#     pickle.dump(samples, f)
# print("Offsets saved to offsets.pkl")


# redoo The 8th offset
var = 0.04
with open("offsets.pkl", "rb") as f:
    offsets = pickle.load(f)
    offsets[19] = np.random.uniform(-var, var, 2)
    print(offsets)

with open("offsets.pkl", "wb") as f:
    pickle.dump(offsets, f)