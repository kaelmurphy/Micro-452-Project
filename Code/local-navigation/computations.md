From the specs of the Thymio cheat sheet (range of speed and refresh rate of the motors) we get the constants :
$$\Delta t = 10 \text{ ms} = \frac{10}{1000} \text{ s}$$
$$v_\text{max mm/s} = 20 \text{ cm/s} = 200 \text{ mm/s}$$
$$v_\text{max lsb/s} = 500 \text{ lsb/s}$$

Taking an initial position estimation in $\text{mm}$ to have a full scale range of $\approx \pm 2^{15} = \pm 32'768 \approx \pm 32 \text{ m}$ (better to have a full scale range larger that a few meters), we get :
$$\hat{x}_\text{mm} = v_\text{lsb/s} \cdot \frac{v_\text{max mm/s}}{v_\text{max lsb}} \cdot \Delta t$$
$$\hat{x}_\text{mm} = v_\text{lsb/s} \cdot \frac{200}{500} \cdot \frac{10}{1000}$$
$$\hat{x}_\text{mm} = v_\text{lsb/s} \cdot \frac{2}{5} \cdot \frac{1}{100}$$
$$\hat{x}_\text{mm} = v_\text{lsb/s} \cdot \frac{2}{500}$$
Because of fixed point arithmetic, only the speeds $250$ and $500$ can be differentiated. They will respectively gives increments of $+1$ and $+2$ on the position estimation, so $\approx 8$ bits of precision are lost.

To take into acount those lost $8$ bits, we must estimate the increment in $\mu m$ :
$$\hat{x}_{\mu m} = 1000 \cdot \hat{x}_\text{mm} = v_\text{lsb/s} \cdot \frac{2}{500} \cdot 1000 = v_\text{lsb/s} \cdot \frac{20}{5} = v_\text{lsb/s} \cdot 4$$

Using this scale, the theoretical resolution of the coders is $4 \, \mu m$. But two signed $16$ bits variables are needed to both store the fine increments in $\mu m$ and have the full scale range in $\text{mm}$.

Summing up both speeds to know the center velocity, we get :
$$\hat{x}_{\mu m} = \frac{v_l + v_r}{2} \cdot 4 = (v_l + v_r) \cdot 2 = (v_l + v_r) << 1$$

With $<<$ representing the left bit shift operator.
