#server logic

from database import test_robot_state, test_location_history, test_key_locations, test_pid_values

# Example entrypoint to manually test all insertions
if __name__ == "__main__":
    test_robot_state.run()
    test_location_history.run()
    test_key_locations.run()
    test_pid_values.run()
