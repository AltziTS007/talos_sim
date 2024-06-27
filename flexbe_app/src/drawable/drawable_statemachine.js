Drawable.Statemachine = function(state_obj, target_paper, readonly, mode, active, locked) {
	var that = this;

	var paper = target_paper;
	var width = 0;
	var height = 60;
	var result = {};
	var state = paper.set();
	var state_obj = state_obj;


	// Title
	//-------

	state_name = paper.text(8, 13, state_obj.getStateName())
		.attr({"text-anchor": 'start', "font-weight": 'bold'});
	if (!readonly) state_name
		.attr({'cursor': 'pointer'})
		.data("state", state_obj)
		.click(Drawable.Helper.viewStateProperties)
		.dblclick(Drawable.Helper.enterStatemachine);
	width = Math.max(width, state_name.getBBox().width);

	state_class = paper.text(8, 28, 
		state_obj.isConcurrent()? "Concurrency":
		state_obj.isPriority()? "Priority": 
		"Statemachine")
		.attr({"text-anchor": 'start', fill: '#555'});
	if (!readonly) state_class
		.attr({'cursor': 'pointer'})
		.data("state", state_obj)
		.click(Drawable.Helper.viewStateProperties)
		.dblclick(Drawable.Helper.enterStatemachine);
	width = Math.max(width, state_class.getBBox().width);

	state_childs = paper.text(8, 41, state_obj.getStates().length + " states")
		.attr({"text-anchor": 'start', fill: '#555'});
	if (!readonly) state_childs
		.attr({'cursor': 'pointer'})
		.data("state", state_obj)
		.click(Drawable.Helper.viewStateProperties)
		.dblclick(Drawable.Helper.enterStatemachine);
	width = Math.max(width, state_childs.getBBox().width);

	state.push(state_name);
	state.push(state_class);
	state.push(state_childs);


	// Outcomes
	//----------

	if (mode == Drawable.State.Mode.OUTCOME) {

		for (i = 0; i < state_obj.getOutcomesUnconnected().length; ++i) {
			if (state_obj.getOutcomesUnconnected()[i].charAt(0) == "$") continue;
			state_oc = paper.text(8, height, state_obj.getOutcomesUnconnected()[i])
				.attr({"text-anchor": 'start', fill: '#005', cursor: 'pointer'})
				.data("state", state_obj)
				.data("label", state_obj.getOutcomesUnconnected()[i])
				.click(Drawable.Helper.beginTransition);
			state.push(state_oc);
			height += 15;
			width = Math.max(width, state_oc.getBBox().width);
		}

	}


	// Mapping
	//---------

	if (mode == Drawable.State.Mode.MAPPING) {

		input_keys = state_obj.getInputKeys();
		input_mapping = state_obj.getInputMapping();
		output_keys = state_obj.getOutputKeys();
		output_mapping = state_obj.getOutputMapping();

		state_im_header = paper.text(5, height, "Input Data:")
			.attr({"text-anchor": 'start'});
		state.push(state_im_header);
		height += 15;
		for (i = 0; i < input_mapping.length; ++i) {
			key = input_keys[i];
			mapping = input_mapping[i];
			state_im = paper.text(5, height, mapping + " (" + key + ")")
				.attr({"text-anchor": 'start', fill: '#050'});
			state.push(state_im);
			height += 15;
			width = Math.max(width, state_im.getBBox().width);
		}
		if (input_mapping.length == 0) {
			state_im = paper.text(5, height, "no input keys")
				.attr({"text-anchor": 'start', fill: '#555', 'font-style': 'italic'});
			state.push(state_im);
			height += 15;
			width = Math.max(width, state_im.getBBox().width);
		}

		height += 5;

		state_om_header = paper.text(5, height, "Output Data:")
			.attr({"text-anchor": 'start'});
		state.push(state_om_header);
		height += 15;
		for (i = 0; i < output_mapping.length; ++i) {
			key = output_keys[i];
			mapping = output_mapping[i];
			state_om = paper.text(5, height, mapping + " (" + key + ")")
				.attr({"text-anchor": 'start', fill: '#500'});
			state.push(state_om);
			height += 15;
			width = Math.max(width, state_om.getBBox().width);
		}
		if (output_mapping.length == 0) {
			state_om = paper.text(5, height, "no output keys")
				.attr({"text-anchor": 'start', fill: '#555', 'font-style': 'italic'});
			state.push(state_om);
			height += 15;
			width = Math.max(width, state_om.getBBox().width);
		}

	}


	// Background
	//------------

	width += 50;
	height -= 2;
	state_outer_box = paper.rect(0, 0, width, height).toBack();
	if (!readonly) state_outer_box
		.attr({'cursor': 'pointer'})
		.data("state", state_obj)
		.click(Drawable.Helper.viewStateProperties)
		.dblclick(Drawable.Helper.enterStatemachine);

	state_box = paper.rect(3, 3, width-6, height-6).toBack();
	if (locked) state_box
		.attr({fill: '120-#eb6:0-#fd9:80', 'stroke-width': 2});
	else if (active) state_box
		.attr({fill: '120-#cde:0-#def:80', 'stroke-width': 2});
	else state_box
		.attr({fill: '120-#eee:0-#fff:80'});
	if (!readonly) state_box
		.attr({'cursor': 'pointer'})
		.data("state", state_obj)
		.click(Drawable.Helper.viewStateProperties)
		.dblclick(Drawable.Helper.enterStatemachine);

	if (!readonly) {
		drag_box = paper.image('img/move-icon.png', width-19, 4, 15, 15)
			.attr({cursor: 'move', 'stroke-width': 1})
			.data("state", state_obj)
			.data("box", state_outer_box)
			.drag(Drawable.Helper.moveFnc, Drawable.Helper.startFnc, Drawable.Helper.endFnc);
		state.push(drag_box);
	}

	state.push(state_box);
	state.push(state_outer_box);
	state.translate(state_obj.getPosition().x, state_obj.getPosition().y);

	this.drawing = state;
	this.obj = state_obj;

	if (!readonly)
		Drawable.Helper.initialIntersectCheck(state, state_obj);
};