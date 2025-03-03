from pysat.solvers import Glucose3
from itertools import combinations
import random
##################################################################################################################

#                           T-Wise Feature interaction

##################################################################################################################
class TWiseInteraction:
    def __init__(self, constraints_cnf, t=2):
        self.constraints_cnf = constraints_cnf
        self.t = t
        

    def _get_valid_t_wise_interactions(self, t_wise_interactions):
        # Filter out combinations with invalid interactions like [-f, f]
        filtered_t_wise_interactions = [
            combination for combination in t_wise_interactions
            if not any(-f in combination for f in combination)
        ]

        # Now check with SAT solver if any of the t_wise interactions are invalid or not from feature model standpoint one after the other
        valid_t_wise_interactions = []
        for interaction in filtered_t_wise_interactions:
            solver = Glucose3(bootstrap_with=self.constraints_cnf)
            
            for literals in interaction:
                solver.add_clause([literals])

            solver_results = solver.solve()
            
            if solver_results:
                # Add only those that have a solution for the feature model constraints
                valid_t_wise_interactions.append(interaction)
            else:
                print(f"{interaction} not satisfied")
            solver.delete()
            del solver

        return valid_t_wise_interactions
    

    def generate_t_wise_interactions(self, features):
        
        # Generate all t-wise combinations of the given features.
        all_t_wise_interactions = list(combinations(features, self.t))
        valid_t_wise_interactions = self._get_valid_t_wise_interactions(all_t_wise_interactions)
        return valid_t_wise_interactions
    
##################################################################################################################

#                           YASA Sampler

##################################################################################################################
class YasaSampler:
    def __init__(self, only_features, constraints_cnf, t, max_samples):
        # List of feature IDs and not the name of the features
        self.only_features = only_features  
        self.total_feature_count = len(self.only_features)
        # Extend self.only_features with their negations
        self.only_features = self.only_features + [-f for f in self.only_features]
        self.constraints_cnf = constraints_cnf  # CNF constraints for the feature model

        self.t = t  # t-wise parameter
        self.max_samples = max_samples  # Maximum number of samples to generate

        self.t_wise_combinations_uncovered = []
        self.sampled_configurations = []  # Store valid configurations
        self.uncovered_combinations = []  # Track uncovered t-wise combinations
        self.sampled_positive_configurations = []
        self.processed_feature_interaction_index = 0
        self.last_configuration_generated = None
        self.t_wise_interaction_generator = TWiseInteraction( self.constraints_cnf, t)
        self.custom_interactions = None

    def set_custom_interactions(self, interactions):
        """Set manual interactions instead of generating t-wise combinations"""
        self.custom_interactions = interactions
    
    def _updating_gives_valid_config(self, config_sample, interaction):
       
        temp_config = config_sample

        #Checking if the newly formed config satisfies the feature model or not
        solver = Glucose3(bootstrap_with=self.constraints_cnf)

        #Combine the config sample and interaction to be considered
        combined_config = temp_config.union(interaction)

        #Adding every literal inside the possible config because every selection/deselection of feature is a constraint in itself 
        for literal in combined_config:
            solver.add_clause([literal])


        if solver.solve():
            # print(f"{interaction} could be merged into {config_sample}")
            solver.delete()
            return True
        else:
            # If theres no solution with the newly formed config then use the original config
            # print(f"{interaction} cannot be merged into {config_sample} because it doesnt satisfy feature model constraints")
            solver.delete()
            return False



    def _update_config_sample_after_checking_coverage(self, configuration_samples, interaction):
        # Check if the new set can be merged into any existing set
        # print(f"Checking interaction {interaction} coverage in configuration sample")
        for current_sample in configuration_samples:
            # Check for literal conflicts, this ensures that we do not merge conflicting values for a given interaction into the configuration sample
            # eg. interaction = {-1, 4}  current_sample = {1,2,3,4,5,6}  => -1 and 1 shall be in conflicting for merging in this case. So we avoid processing such cases beforehand
            if not any(-x in current_sample for x in interaction):
                # using SETS simplifies the case of interaction being already covered as during merge no duplicates get added anyways
                if (self._updating_gives_valid_config(current_sample, interaction)):
                    # if its able to solve then the newly formed config should be considered
                    current_sample.update(interaction)
                    return configuration_samples  # Return updated list
        
        # If no match is found, add new set to the list
        # print(f"No suitable configuration sample found creating new configuration as {interaction}")
        configuration_samples.append(interaction)
        return configuration_samples

    # t:total feature counts
    def trim_partial_config_sample(self, configuration_samples, t_wise_interactions, t):
        print(f"total features {t}")
        # Convert configurations and interactions to sets for easier comparison
        if configuration_samples:
            config_sets = [set(c) for c in configuration_samples]
            interaction_sets = [set(i) for i in t_wise_interactions]
            ranks = []

            # Calculate rank for each configuration
            for config in config_sets:
                unique_coverage_count = 0
                for interaction in interaction_sets:
                    if interaction.issubset(config):  # Interaction is covered only by this config
                        is_unique = all(
                            not interaction.issubset(other_config) or config == other_config
                            for other_config in config_sets
                        )
                        if is_unique:
                            unique_coverage_count += 1
                # Calculate the rank for given configuration under consideration
                rank = unique_coverage_count / (len(config) ** t)
                ranks.append(rank)

            # Compute the arithmetic mean of ranks
            mean_rank = sum(ranks) / len(ranks)

            print(f"Ranks: {ranks}")
            print(f"Mean rank: {mean_rank}")
            # Filter configurations with rank >= mean_rank
            trimmed_configs = [
                configuration_samples[i] for i, rank in enumerate(ranks) if rank >= mean_rank
            ]

            print("-------------------------------------------TRIMMED SAMPLE SPACE--------------------------------------------")
            for conf in trimmed_configs:
               print(conf)

            return trimmed_configs

    def generate_partial_config_samples(self, t_wise_interaction_original_copy):
        configuration_samples = []
        sample_space = []
        t_wise_interaction = [set(s) for s in t_wise_interaction_original_copy]

        for m in range(self.max_samples):
            #Re-load the interactions
            t_wise_interaction = t_wise_interaction_original_copy
            random.shuffle(t_wise_interaction)
            t_wise_interaction = [set(s) for s in t_wise_interaction]
            # Use the trimmed sample space as starting point for generating new config space with feature interactions
            configuration_samples = sample_space.copy()
            for interaction in t_wise_interaction:
                configuration_samples = self._update_config_sample_after_checking_coverage(configuration_samples, interaction)
                

           

            # Uncomment this for the trimming sample code
            #Once we have the configuration sample, we shall try to trim it and do this repeatedly for m times of sampling
            trimmed_samples = self.trim_partial_config_sample(configuration_samples, t_wise_interaction_original_copy, self.total_feature_count)
            sample_space.extend(trimmed_samples)

            #Ensuring to keep unique configurations only
            unique_sample_space = []
            for s in sample_space:
                if s not in unique_sample_space:
                    unique_sample_space.append(s)
            sample_space = unique_sample_space

            print(f"----------------------Current Partial Sample space--------------------------")
            for config in sample_space:
                print(config)


        configuration_samples = sample_space.copy()
        return configuration_samples

    def get_missing_literals(self, config_sample):
        # Convert to absolute values first and then find the which literal is missing
        config_sample = [abs(x) for x in config_sample]
        missing_literals = list(set(self.only_features) - set(config_sample))
        print(f"Missing literals :{missing_literals}")
        return missing_literals

    def complete_partial_configurations(self, partial_config_samples):
        complete_configurations = []

        # For all the configs in the list complete the configuration using a SAT solver to assign values for the remaining features
        for partial_configuration in partial_config_samples:
            solver = Glucose3(bootstrap_with=self.constraints_cnf)
            # missing_literals = self.get_missing_literals(partial_configuration)

            #Update the constraint to one of the partial_configuration 
            # solver.add_clause(partial_configuration)
            for literal in partial_configuration:
                solver.add_clause([literal])

            solver_results = solver.solve()
            if solver_results:
                # If a solution is found add it to the complete config list
                completed_config_solution = solver.get_model()
                print(f"Generated configuration:  {completed_config_solution}")

                # First check if its an unique configuration or not
                if not (completed_config_solution in  complete_configurations):
                    complete_configurations.append(completed_config_solution)
                else:
                    print(f"Configuration repeated, Skipping.....")
            else:
                print(f"No solution can be formed using the partial configuration {partial_configuration} as completed configuration")
                complete_configurations.append(partial_configuration)


            #once done with solution we need to release the resources to start the new solver
            solver.delete()
            del solver
        
        return complete_configurations

    def run_coverage_analysis(self, completed_configurations, t_wise_interations):

        #First transform all configs and interactions into set for easier coverage analysis
        completed_configurations_set = [set(x) for x in completed_configurations]
        t_wise_interations_set = [set(x) for x in t_wise_interations]

        for configs in completed_configurations_set:
            for interaction in t_wise_interations_set[:]:  # Use slice to copy list
                # Check if interaction is a subset of the current configuration
                if interaction.issubset(configs):
                    t_wise_interations_set.remove(interaction)  # Remove the covered interaction

        return len(t_wise_interations_set)

    def get_features_to_select_in_config(self, completed_config_samples):
        return [[num for num in sublist if num > 0] for sublist in completed_config_samples]


    def start_sampling(self):

        if self.custom_interactions:
            self.t_wise_interactions = self.custom_interactions
        else:
            self.t_wise_interactions = self.t_wise_interaction_generator.generate_t_wise_interactions(self.only_features)

        # Add print statements here
        print("\n=== T-wise Feature Interactions ===")
        print(f"T = {self.t}")
        for interaction in self.t_wise_interactions:
            print(f"Interaction: {interaction}")
        print(f"Total interactions: {len(self.t_wise_interactions)}\n")

        partial_config_samples = self.generate_partial_config_samples(self.t_wise_interactions)

        # Above operation has been designed to operate on SETs to make implementation simpler. Convert them back to lists
        partial_config_samples = [list(l) for l in partial_config_samples]

        print(f"----------------------Partial configuration samples--------------------------")
        for config in partial_config_samples:
            print(config)

        completed_config_samples = self.complete_partial_configurations(partial_config_samples)

        print(f"----------------------Completed configuration samples--------------------------")
        for config in completed_config_samples:
            print(config)

        uncovered_interactions = self.run_coverage_analysis(completed_config_samples, self.t_wise_interactions)
        print(f"Uncovered interactions : {uncovered_interactions}")

        completed_config_samples_only_selections = self.get_features_to_select_in_config(completed_config_samples)
        print(f"----------------------Features to select--------------------------")
        for config in completed_config_samples_only_selections:
            print(config)
    
        #Ensure to return only the positive IDs that should be selected
        return completed_config_samples_only_selections




#Testing code
only_features = [1, 2, 3, 4, 5]  # Example feature IDs
constraints_cnf = [
    [-1, 2],  # A requires B
    [-3, 4],  # C requires D
    [-5, -1]  # E excludes A
]
t = 2  # Pairwise sampling


constraints_cnf = [
    [24],
    [3, -17],
    [3, -6],
    [17, -3],
    [6, -3],
    [6, -7],
    [6, -21],
    [7, -6],
    [21, -6],
    [8, -19],
    [9, -8],
    [9, -27],
    [9, -22],
    [10, -2],
    [10, -5],
    [10, -12],
    [10, -11],
    [10, -23],
    [10, -25],
    [2, 5, 12, 11, 23, 25, -10],
    [16, -10],
    [17, -18],
    [17, -20],
    [18, -17],
    [21, -4],
    [21, -13],
    [4, 13, -21],
    [-4, -13],
    [22, -1],
    [24, -16],
    [24, -3],
    [24, -9],
    [16, -24],
    [3, -24],
    [27, -26],
    [27, -28],
    [27, -14],
    [27, -15],
    [-11, 22],
    [-25, 20],
    [-26, -4],
    [-23, 8],
    [-27, 8],
    [-15, 22] 
]

print(f"Updated constraints : {constraints_cnf}")

only_features = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
# Initialize the sampler
sampler = YasaSampler(only_features, constraints_cnf, t=2, max_samples=1000)

# Set custom interactions
manual_interactions = [

    (4 , 8),
    (13 , 8),
    (17 , 12),
    (17 , 20),
    (12 , 20),
    (22 , 15),
    (22 , 1),
    (15 , 1),
    (27 , 26),
    (27 , 28),
    (26 , 28),
    (8 , 19),
    (8 , 27),
    (27 , 19),
    (5 , 4),
    (5 , 13),
    (5 , 8),
    (11 , 22),
    (17 , 25),
    (4 , 2),
    (13 , 2),
    (8 , 23)

]
sampler.set_custom_interactions(manual_interactions)


# Start t-wise sampling
samples = sampler.start_sampling()

