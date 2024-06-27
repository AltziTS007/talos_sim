State = function(state_name, state_def) {
	var that = this;

	var updateGeneratedOutcomes = function(new_vals) {
		var resolved_parameter_outcome_values = [];
		if (meta_outcomes.length == 0) return;
		meta_outcomes.forEach(function(moc, i) {
			var param_key = parameters.findElement(function(element) {
				return element == moc;
			});
			var pidx = parameters.indexOf(param_key);
			var param_val_old = (resolved_parameter_outcome_values_old.length > i)? resolved_parameter_outcome_values_old[i] : [];
			var param_val_new = (new_vals.length > pidx)? VarSolver.getStringValues(new_vals[pidx], true) : [];

			if (param_val_new === false) {
				T.logWarn("Only strings or a list of strings is allowed as outcomes.");
				param_val_new = [];
			}
			resolved_parameter_outcome_values.push(param_val_new);

			var remove_list = param_val_old.filter(function(element) {
				return !param_val_new.contains(element);
			});
			var add_list = param_val_new.filter(function(element) {
				return !param_val_old.contains(element);
			});

			remove_list.forEach(function(element) {
				var idx = outcomes.indexOf(element);
				outcomes.splice(idx, 1);
				autonomy.splice(idx, 1);
				if (outcomes_unc.contains(element)) {
					outcomes_unc.remove(element);
				} else if (outcomes_con.contains(element)) {
					outcomes_con.remove(element);
					if (container != undefined) {
						container.removeTransitionFrom(that, element);
					}
				}
			});
			add_list.forEach(function(element) {
				if (outcomes.contains(element)) {
					T.logWarn("Adding an already existing outcome!");
				}
				outcomes.push(element);
				autonomy.push(0);
				outcomes_unc.push(element);
			});
		});
		resolved_parameter_outcome_values_old = resolved_parameter_outcome_values;
	}

	var updateGeneratedInput = function(new_vals) {
		var resolved_parameter_input_values = [];
		if (meta_input.length == 0) return;
		meta_input.forEach(function(mik, i) {
			var param_key = parameters.findElement(function(element) {
				return element == mik;
			});
			var pidx = parameters.indexOf(param_key);
			var param_val_old = (resolved_parameter_input_values_old.length > i)? resolved_parameter_input_values_old[i] : [];
			var param_val_new = (new_vals.length > pidx)? VarSolver.getStringValues(new_vals[pidx], true) : [];

			if (param_val_new === false) {
				T.logWarn("Only strings or a list of strings is allowed as input keys.");
				param_val_new = [];
			}
			resolved_parameter_input_values.push(param_val_new);

			var remove_list = param_val_old.filter(function(element) {
				return !param_val_new.contains(element);
			});
			var add_list = param_val_new.filter(function(element) {
				return !param_val_old.contains(element);
			});

			remove_list.forEach(function(element) {
				var idx = input_keys.indexOf(element);
				input_keys.splice(idx, 1);
				input_mapping.splice(idx, 1);
			});
			add_list.forEach(function(element) {
				if (input_keys.contains(element)) {
					T.logWarn("Adding an already existing input key!");
				}
				input_keys.push(element);
				input_mapping.push(element);
			});
		});
		resolved_parameter_input_values_old = resolved_parameter_input_values;
	}

	var updateGeneratedOutput = function(new_vals) {
		var resolved_parameter_output_values = [];
		if (meta_output.length == 0) return;
		meta_output.forEach(function(mok, i) {
			var param_key = parameters.findElement(function(element) {
				return element == mok;
			});
			var pidx = parameters.indexOf(param_key);
			var param_val_old = (resolved_parameter_output_values_old.length > i)? resolved_parameter_output_values_old[i] : [];
			var param_val_new = (new_vals.length > pidx)? VarSolver.getStringValues(new_vals[pidx], true) : [];

			if (param_val_new === false) {
				T.logWarn("Only strings or a list of strings is allowed as output keys.");
				param_val_new = [];
			}
			resolved_parameter_output_values.push(param_val_new);

			var remove_list = param_val_old.filter(function(element) {
				return !param_val_new.contains(element);
			});
			var add_list = param_val_new.filter(function(element) {
				return !param_val_old.contains(element);
			});

			remove_list.forEach(function(element) {
				var idx = output_keys.indexOf(element);
				output_keys.splice(idx, 1);
				output_mapping.splice(idx, 1);
			});
			add_list.forEach(function(element) {
				if (output_keys.contains(element)) {
					T.logWarn("Adding an already existing output key!");
				}
				output_keys.push(element);
				output_mapping.push(element);
			});
		});
		resolved_parameter_output_values_old = resolved_parameter_output_values;
	}

	var noMetaFilter = function(element) { return element[0] != '$'; };
	var metaFilter = function(element) { return element[0] == '$'; };

	var state_name = state_name;
	var state_class = state_def.getStateClass();
	var state_import = state_def.getStatePath();
	var state_pkg = state_def.getStatePackage();
	var behavior = undefined;

	var parameters = state_def.getParameters().clone();
	var parameter_values = state_def.getDefaultParameterValues().clone();

	var outcomes = state_def.getOutcomes().filter(noMetaFilter);
	var autonomy = state_def.getDefaultAutonomy().clone();

	var meta_outcomes = state_def.getOutcomes().filter(metaFilter);
	for (var i=0; i<meta_outcomes.length; ++i) meta_outcomes[i] = meta_outcomes[i].slice(1, meta_outcomes[i].length);

	var outcomes_unc = outcomes.clone();
	var outcomes_con = [];

	var input_keys = state_def.getInputKeys().filter(noMetaFilter);
	var output_keys = state_def.getOutputKeys().filter(noMetaFilter);

	var meta_input = state_def.getInputKeys().filter(metaFilter);
	for (var i=0; i<meta_input.length; ++i) meta_input[i] = meta_input[i].slice(1, meta_input[i].length);

	var meta_output = state_def.getOutputKeys().filter(metaFilter);
	for (var i=0; i<meta_output.length; ++i) meta_output[i] = meta_output[i].slice(1, meta_output[i].length);

	var input_mapping = [];
	for (var i=0; i<input_keys.length; ++i) input_mapping[i] = input_keys[i];

	var output_mapping = [];
	for (var i=0; i<output_keys.length; ++i) output_mapping[i] = output_keys[i];

	var position = {x: 30, y: 40};

	var container = undefined;

	var resolved_parameter_outcome_values_old = [];
	var resolved_parameter_input_values_old = [];
	var resolved_parameter_output_values_old = [];

	updateGeneratedOutcomes([], parameter_values);

	this.getStateName = function() {
		if (behavior != undefined) {
			return behavior.getStateName();
		} else {
			return state_name;
		}
	}
	this.setStateName = function(_state_name) {
		if (behavior != undefined) {
			behavior.setStateName(_state_name);
		}
		if (container != undefined && container.getStateByName(_state_name) != undefined) {
			T.logWarn("Renaming state failed, name already in use!");
			return;
		}
		state_name = _state_name;
	}

	this.getStatePath = function() {
		return ((container != undefined || behavior != undefined)? that.getContainer().getStatePath() + "/" : "")
			+ that.getStateName();
	}

	this.getStateClass = function() {
		return state_class;
	}
	this.setStateClass = function(_state_class) {
		state_class = _state_class;
	}

	this.getStateImport = function() {
		return state_import;
	}
	this.setStateImport = function(_state_import) {
		state_import = _state_import;
	}

	this.getStatePackage = function() {
		return state_pkg;
	}
	this.setStatePackage = function(_state_pkg) {
		state_pkg = _state_pkg;
	}

	this.getStateType = function() {
		return that.getStatePackage() + "." + that.getStateClass();
	}

	this.getParameters = function() {
		return parameters;
	}
	this.setParameters = function(_parameters) {
		parameters = _parameters;
	}

	this.getParameterValues = function() {
		return parameter_values;
	}
	this.setParameterValues = function(_parameter_values) {
		updateGeneratedOutcomes(_parameter_values);
		updateGeneratedInput(_parameter_values);
		updateGeneratedOutput(_parameter_values);
		parameter_values = _parameter_values;
	}

	this.getOutcomes = function() {
		return outcomes;
	}
	this.setOutcomes = function(_outcomes) {
		outcomes = _outcomes;
	}

	this.getAutonomy = function() {
		return autonomy;
	}
	this.setAutonomy = function(_autonomy) {
		autonomy = _autonomy;
	}

	this.getGeneratedAutonomy = function() {
		return generated_autonomy;
	}
	this.setGeneratedAutonomy = function(_generated_autonomy) {
		generated_autonomy = _generated_autonomy;
	}

	this.getCombinedAutonomy = function() {
		return autonomy.concat(generated_autonomy);
	}

	this.getOutcomesUnconnected = function() {
		return outcomes_unc;
	}
	this.setOutcomesUnconnected = function(_outcomes_unc) {
		outcomes_unc = _outcomes_unc;
	}

	this.getInputKeys = function() {
		return input_keys;
	}
	this.setInputKeys = function(_input_keys) {
		input_keys = _input_keys;
		var input_mapping = [];
		for (var i=0; i<input_keys.length; ++i) input_mapping.push(input_keys[i]);
	}

	this.getOutputKeys = function() {
		return output_keys;
	}
	this.setOutputKeys = function(_output_keys) {
		output_keys = _output_keys;
		var output_mapping = [];
		for (var i=0; i<output_keys.length; ++i) output_mapping.push(output_keys[i]);
	}

	this.getInputMapping = function() {
		return input_mapping;
	}
	this.setInputMapping = function(_input_mapping) {
		input_mapping = _input_mapping;
	}

	this.getOutputMapping = function() {
		return output_mapping;
	}
	this.setOutputMapping = function(_output_mapping) {
		output_mapping = _output_mapping;
	}

	this.getOutcomesConnected = function() {
		return outcomes_con;
	}
	this.setOutcomesConnected = function(_outcomes_con) {
		outcomes_con = _outcomes_con;
	}

	this.getPosition = function() {
		return position;
	}
	this.setPosition = function(_position) {
		position = _position;
	}

	this.getContainer = function() {
		return (behavior != undefined && container == undefined)? behavior.getContainer() : container;
	}
	this.setContainer = function(_container) {
		container = _container;
	}

	this.connect = function (outcome) {
		if (!outcomes_unc.contains(outcome)) {
			T.debugWarn("Trying to connect unavailable outcome '" + outcome + "' of state " + that.getStateName());
			return;
		}
		outcomes_unc.remove(outcome);
		outcomes_con.push(outcome);
	}
	this.unconnect = function (outcome) {
		if (!outcomes_con.contains(outcome)) {
			T.debugWarn("Trying to disconnect unavailable outcome '" + outcome + "' of state " + that.getStateName());
			return;
		}
		outcomes_unc.push(outcome);
		outcomes_con.remove(outcome);
	}

	this.translate = function (dx, dy) {
		position.x += dx;
		position.y += dy;
	}

	this.updatePosition = function() {
		drawing = UI.Statemachine.getDrawnState(this);
		drawing.attr({x: position.x, y: position.y});
	}

	this.isInsideDifferentBehavior = function() {
		return that.getBehavior() != undefined || (that.getContainer() != undefined && that.getContainer().isInsideDifferentBehavior());
	}

	this.setBehavior = function(behavior_state) {
		behavior = behavior_state;
	}
	this.getBehavior = function() {
		return behavior;
	}

	this.updateStateDefinition = function(new_def) {
		state_class = new_def.getStateClass();
		state_import = new_def.getStatePath();
		state_pkg = new_def.getStatePackage();

		var updateKeys = function(old_keys, new_keys, old_values, new_values) {
			var result = {keys: [], values: []};
			for (var i=0; i<new_keys.length; i++) {
				var j = old_keys.indexOf(new_keys[i]);
				if (j != -1) {
					result.keys.push(old_keys[j]);
					result.values.push(old_values[j]);
				} else {
					result.keys.push(new_keys[i]);
					result.values.push(new_values[i]);
				}
			}
			return result;
		}

		// required for meta
		var old_params = parameters;

		if (!parameters.hasSameElements(new_def.getParameters())) {
			var result = updateKeys(parameters, new_def.getParameters(), parameter_values, new_def.getDefaultParameterValues());
			parameters = result.keys;
			parameter_values = result.values;
		}

		if (!outcomes.hasSameElements(new_def.getOutcomes().filter(noMetaFilter))) {
			var old_outcomes = outcomes;
			var result = updateKeys(outcomes, new_def.getOutcomes().filter(noMetaFilter), autonomy, new_def.getDefaultAutonomy());
			outcomes = result.keys;
			autonomy = result.values;
			for (var i=0; i<outcomes.length; i++) {
				if (!old_outcomes.contains(outcomes[i])) {
					outcomes_unc.push(outcomes[i]);
				}
			}
			for (var i=0; i<old_outcomes.length; i++) {
				if (!outcomes.contains(old_outcomes[i])) {
					if (outcomes_con.contains(old_outcomes[i])) {
						if (container != undefined) {
							container.removeTransitionFrom(that, old_outcomes[i]);
						}
						outcomes_con.remove(old_outcomes[i]);
					} else {
						outcomes_unc.remove(old_outcomes[i]);
					}
				}
			}
		}

		var new_meta_outcomes = new_def.getOutcomes().filter(metaFilter);
		for (var i=0; i<new_meta_outcomes.length; ++i) new_meta_outcomes[i] = new_meta_outcomes[i].slice(1, new_meta_outcomes[i].length);
		if (!meta_outcomes.hasSameElements(new_meta_outcomes)) {
			for (var i=0; i<meta_outcomes.length; i++) {
				if (!new_meta_outcomes.contains(meta_outcomes[i])) {
					var pidx = old_params.indexOf(meta_outcomes[i]);
					var remove_list = resolved_parameter_outcome_values_old[i];
					remove_list.forEach(function(element) {
						var idx = outcomes.indexOf(element);
						outcomes.splice(idx, 1);
						autonomy.splice(idx, 1);
						if (outcomes_unc.contains(element)) {
							outcomes_unc.remove(element);
						} else if (outcomes_con.contains(element)) {
							outcomes_con.remove(element);
							if (container != undefined) {
								container.removeTransitionFrom(that, element);
							}
						}
					});
				}
			}
			meta_outcomes = new_meta_outcomes;
			updateGeneratedOutcomes(parameter_values);
		}

		if (!input_keys.hasSameElements(new_def.getInputKeys().filter(noMetaFilter))) {
			var result = updateKeys(input_keys, new_def.getInputKeys().filter(noMetaFilter), input_mapping, new_def.getInputKeys().filter(noMetaFilter));
			input_keys = result.keys;
			input_mapping = result.values;
		}

		var new_meta_input = new_def.getInputKeys().filter(metaFilter);
		for (var i=0; i<new_meta_input.length; ++i) new_meta_input[i] = new_meta_input[i].slice(1, new_meta_input[i].length);
		if (!meta_input.hasSameElements(new_meta_input)) {
			for (var i=0; i<meta_input.length; i++) {
				if (!new_meta_input.contains(meta_input[i])) {
					var pidx = old_params.indexOf(meta_input[i]);
					var remove_list = resolved_parameter_input_values_old[i];
					remove_list.forEach(function(element) {
						var idx = input_keys.indexOf(element);
						input_keys.splice(idx, 1);
						input_mapping.splice(idx, 1);
					});
				}
			}
			meta_input = new_meta_input;
			updateGeneratedInput(parameter_values);
		}

		if (!output_keys.hasSameElements(new_def.getOutputKeys().filter(noMetaFilter))) {
			var result = updateKeys(output_keys, new_def.getOutputKeys().filter(noMetaFilter), output_mapping, new_def.getOutputKeys().filter(noMetaFilter));
			output_keys = result.keys;
			output_mapping = result.values;
		}

		var new_meta_output = new_def.getOutputKeys().filter(metaFilter);
		for (var i=0; i<new_meta_output.length; ++i) new_meta_output[i] = new_meta_output[i].slice(1, new_meta_output[i].length);
		if (!meta_output.hasSameElements(new_meta_output)) {
			for (var i=0; i<meta_output.length; i++) {
				if (!new_meta_output.contains(meta_output[i])) {
					var pidx = old_params.indexOf(meta_output[i]);
					var remove_list = resolved_parameter_output_values_old[i];
					remove_list.forEach(function(element) {
						var idx = output_keys.indexOf(element);
						output_keys.splice(idx, 1);
						output_mapping.splice(idx, 1);
					});
				}
			}
			meta_output = new_meta_output;
			updateGeneratedOutput(parameter_values);
		}
	}

};