import numpy as np
from scipy import stats

def detailed_ci_calculation(data, confidence=0.95):
    """
    Detailed breakdown of confidence interval calculation
    """
    print(f"Data: {data}")
    print(f"Confidence level: {confidence * 100}%")
    print("-" * 50)
    
    # Step 1: Calculate sample statistics
    n = len(data)
    mean = np.mean(data)
    std = np.std(data, ddof=1)  # Sample standard deviation (n-1 denominator)
    
    print(f"1. Sample size (n): {n}")
    print(f"2. Sample mean: {mean:.4f}")
    print(f"3. Sample standard deviation: {std:.4f}")
    
    # Step 2: Calculate standard error of the mean
    sem = std / np.sqrt(n)  # This is what stats.sem() does
    print(f"4. Standard error of mean (SEM): {std:.4f} / √{n} = {sem:.4f}")
    
    # Step 3: Find the critical t-value
    alpha = 1 - confidence  # e.g., 0.05 for 95% confidence
    df = n - 1  # degrees of freedom
    t_critical = stats.t.ppf((1 + confidence) / 2., df)
    
    print(f"5. Alpha (α): {alpha}")
    print(f"6. Degrees of freedom: {df}")
    print(f"7. Critical t-value (two-tailed): {t_critical:.4f}")
    
    # Step 4: Calculate margin of error
    margin_of_error = t_critical * sem
    print(f"8. Margin of error: {t_critical:.4f} × {sem:.4f} = {margin_of_error:.4f}")
    
    # Step 5: Calculate confidence interval bounds
    ci_lower = mean - margin_of_error
    ci_upper = mean + margin_of_error
    
    print(f"9. Confidence interval: [{ci_lower:.4f}, {ci_upper:.4f}]")
    print(f"10. Error bar size (±): {margin_of_error:.4f}")
    
    return mean, margin_of_error

# Example with your phase_1 basic task data
example_data = [24/25, 25/25, 24/25, 23/25, 25/25]  # success rates
print("Example: Phase 1 Basic Task Success Rates")
print("=" * 60)
detailed_ci_calculation(example_data, confidence=0.95)

print("\n" + "="*60)
print("COMPARISON: Different Confidence Levels")
print("="*60)

for conf_level in [0.90, 0.95, 0.99]:
    mean, margin = detailed_ci_calculation(example_data, confidence=conf_level)
    print(f"\n{conf_level*100}% CI: {mean:.3f} ± {margin:.3f}")