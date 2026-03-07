import matplotlib.pyplot as plt

def solve_sleep_probability():
    # 1. Total Sample Space Area (Omega)
    # Bounded by 0 <= t1 <= 24, 0 <= t2 <= 24, and t2 > t1
    # This is a triangle with vertices (0,0), (24,24), (0,24)
    total_area_omega = 0.5 * 24 * 24

    # 2. Event A: "student is asleep at noon"
    # Occurs if t1 > 6 (wakes up after noon) OR t2 < 6 (sleeps before noon)
    
    # Area where student wakes up after noon (t1 > 6 and t2 > t1)
    # Triangle with vertices (6,6), (24,24), (6,24)
    base_t1_gt_6 = 24 - 6
    height_t1_gt_6 = 24 - 6
    area_t1_gt_6 = 0.5 * base_t1_gt_6 * height_t1_gt_6

    # Area where student goes to sleep before noon (t2 < 6 and t2 > t1)
    # Triangle with vertices (0,0), (6,6), (0,6)
    base_t2_lt_6 = 6 - 0
    height_t2_lt_6 = 6 - 0
    area_t2_lt_6 = 0.5 * base_t2_lt_6 * height_t2_lt_6

    # Total area of event A
    area_event_A = area_t1_gt_6 + area_t2_lt_6

    # 3. Calculate Probability P(A) = Area(A) / Area(Omega)
    probability_A = area_event_A / total_area_omega

    print(f"Area of Sample Space (Ω): {total_area_omega}")
    print(f"Area of Event A (Asleep at noon): {area_event_A}")
    print(f"Probability P(A): {probability_A:.4f} (or {int(area_event_A)}/{int(total_area_omega)} = 5/8)\n")
    
    # 4. Sketch the region
    plt.figure(figsize=(8, 8))
    
    # Plot Sample Space Omega
    plt.fill([0, 24, 0], [0, 24, 24], color='#e0e0e0', label='Sample Space $\Omega$ ($t_2 > t_1$)')
    
    # Plot Event A (t1 > 6)
    plt.fill([6, 24, 6], [6, 24, 24], color='skyblue', alpha=0.8, label='Event A: Wakes after noon ($t_1 > 6$)')
    
    # Plot Event A (t2 < 6)
    plt.fill([0, 6, 0], [0, 6, 6], color='salmon', alpha=0.8, label='Event A: Sleeps before noon ($t_2 < 6$)')
    
    # Draw reference lines for noon
    plt.axvline(x=6, color='gray', linestyle='--', linewidth=1)
    plt.axhline(y=6, color='gray', linestyle='--', linewidth=1)
    plt.text(6.2, 1, 'Noon ($t_1=6$)', rotation=90, color='gray')
    plt.text(1, 6.2, 'Noon ($t_2=6$)', color='gray')

    # Formatting
    plt.xlim(0, 24)
    plt.ylim(0, 24)
    plt.xlabel('Wake up time, $t_1$ (hours after 6 AM)')
    plt.ylabel('Sleep time, $t_2$ (hours after 6 AM)')
    plt.title('Region for Event A: "Student is asleep at noon"')
    plt.legend(loc='upper left')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.gca().set_aspect('equal', adjustable='box')
    
    print("Close the plot window to finish the script.")
    plt.show()

if __name__ == "__main__":
    solve_sleep_probability()