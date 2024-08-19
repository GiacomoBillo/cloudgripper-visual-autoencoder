
import os
import json
import numpy as np

# Define epsilon for comparison
EPSILON = 1e-2

def load_json(file_path):
    """Load JSON data from a file."""
    with open(file_path, 'r') as file:
        return json.load(file)

def compare_values(val1, val2, epsilon=EPSILON):
    """Compare two values with a given epsilon."""
    return np.abs(val1 - val2) < epsilon

def find_matching_state(order, states, start_idx):
    """Find the first state where the order is successfully carried out, starting from start_idx."""
    order_type = order['order_type']
    order_value = order.get('order_value', [])

    for idx in range(start_idx, len(states)):
        state = states[idx]
        
        if order_type == "MOVE_XY":
            if len(order_value) == 2 and (
                compare_values(order_value[0], state['x_norm']) and
                compare_values(order_value[1], state['y_norm'])):
                return idx
        
        elif order_type == "MOVE_Z":
            if len(order_value) == 1 and compare_values(order_value[0], state['z_norm']):
                return idx

        elif order_type == "GRIPPER_OPEN":
            # Match with claw_norm close to 1.0
            if compare_values(1.0, state['claw_norm']):
                return idx

        elif order_type == "GRIPPER_CLOSE":
            # Match with claw_norm close to 0.21
            if compare_values(0.21, state['claw_norm']):
                return idx

    return None

def process_task(task_dir):
    """Process each task directory."""
    states_file = os.path.join(task_dir, 'states.json')
    orders_file = os.path.join(task_dir, 'orders.json')

    # Load data
    states = load_json(states_file)
    orders = load_json(orders_file)

    matching_states = []
    last_matched_index = None

    for order in orders:
        match_index = find_matching_state(order, states, last_matched_index + 1 if last_matched_index is not None else 0)
        if match_index is not None:
            if last_matched_index is not None and match_index == last_matched_index + 1:
                # Skip this match as it is consecutive to the last one
                continue
            matching_states.append((order, match_index, states[match_index]))  # Store order, index, and state
            last_matched_index = match_index
        else:
            break  # If an order can't be matched, stop further processing

    return matching_states

def post_process_results(results):
    """Shift orders one index back and remove the first order and last state."""
    if len(results) > 1:
        # Shift orders back by one index
        shifted_results = [(results[i][0], results[i-1][1], results[i-1][2]) for i in range(1, len(results))]

        # Remove the first order and last state
        shifted_results.pop()  # Remove the last element (last state)
        return shifted_results
    return results  # If results length is 1 or less, return as is

def save_results(task_dir, results):
    """Save the results to a JSON file in the specified format."""
    output = []
    for order, _, state in results:
        combined_entry = {
            "x_norm": state['x_norm'],
            "y_norm": state['y_norm'],
            "z_norm": state['z_norm'],
            "rotation": state['rotation'],
            "claw_norm": state['claw_norm'],
            "order_type": order['order_type'],
            "order_value": order['order_value']
        }
        output.append(combined_entry)
    
    output_file = os.path.join(task_dir, 'state_order_combined.json')
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=4)

def main(root_dir):
    """Main function to iterate over all tasks."""
    base_dir = os.path.join(root_dir, 'stack_from_scratch', 'recorded_data')
    
    task_numbers = sorted(os.listdir(base_dir))
    if task_numbers:
        first_task_number = task_numbers[0]
        task_dir = os.path.join(base_dir, first_task_number, 'task')
        if os.path.isdir(task_dir):
            matching_states = process_task(task_dir)
            post_processed_results = post_process_results(matching_states)
            save_results(task_dir, post_processed_results)

if __name__ == "__main__":
    root_dir = "."
    main(root_dir)

