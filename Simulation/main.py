from scripts.simulation import Simulation
import argparse

def main():

    parser = argparse.ArgumentParser(description="Reverse Manoeuvre Simulator")
    parser.add_argument(
        '--config', 
        type=str, 
        default='config.yaml', 
        help='Path to the YAML configuration file'
    )
    
    args = parser.parse_args()

    print(f"--- Starting Simulation with {args.config} ---")
    
    # Initialize and run
    sim = Simulation(config_path=args.config)
    
    # Because your UI loop is blocking, the script will stay alive here
    print("Simulation ended.")

if __name__ == "__main__":
    main()