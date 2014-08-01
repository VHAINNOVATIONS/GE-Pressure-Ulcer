function BradenDialog() {
	this.submitLabel = "Create a new set of scores";
	this.dialog = $( '#braden-scores-form-div' ).dialog({
		autoOpen: false,
		height: 600,
		width: 350,
		modal: true,
		buttons: [
			{text: this.submitLabel, click : this.bradenDialogSubmit()},
			{text: "Cancel", click: function() { $(this).dialog("close"); } }
		],
		close: function() {
			$('form[name="braden-scores-form"]')[0].reset();
			// allFields.removeClass( "ui-state-error" );
		}
	});
	this.confirmDialog = $( '#braden-scores-delete-div' ).dialog({
		autoOpen: false,
		height: 150,
		width: 400,
		modal: true,
		buttons: [
			{text: "Yes", click : this.bradenDialogDelete()},
			{text: "No", click: function() { $(this).dialog("close"); } }
		],
		close: function() {	$(this).dialog("close"); }
	});
	$('#braden_scoring_date').datetimepicker({format:'Y-m-d H:i:s', step:30});
	$('#braden-scores-form').validate({
		debug: true,
		rules: {
			"braden_scoring_date": {required: true},
			"sensory_perception_score": {required: true, digits: true, range: [1,4]},
			"moisture_score": {required: true, digits: true, range: [1,4]},
			"activity_score": {required: true, digits: true, range: [1,4]},
			"mobility_score": {required: true, digits: true, range: [1,4]},
			"nutrition_score": {required: true, digits: true, range: [1,4]},
			"friction_shear_score": {required: true, digits: true, range: [1,3]}
		},
		errorPlacement: function(error, element) {
			// Append error within linked label
			$( element )
				.closest( "form" )
					.find( "label[for='" + element.attr( "id" ) + "']" )
						.append( error );
		}
	})
	this.dialog.data("myData", {mode: "", idx: -1, table: this.table})
}

BradenDialog.prototype.bradenDialogSubmit = function () {
	var self = this;
	var d = $(this).data("myData");
	if (!(typeof d === 'undefined')) {
		self.mode = d.mode;
		self.idx = d.idx;
		self.table = d.table;		
		self.dataTable = d.dataTable;		
	} else {
		return;
	}
	self.patient_id = parseInt($('#braden-scores-form input[name="patient_id"]').val());
	self.updatedRow = {};
	self.updatedRow.braden_scoring_date = $('#braden-scores-form input[name="braden_scoring_date"]').val();
	self.updatedRow.sensory_perception_score = parseInt($('#braden-scores-form input[name="sensory_perception_score"]').val());
	self.updatedRow.moisture_score = parseInt($('#braden-scores-form input[name="moisture_score"]').val());
	self.updatedRow.activity_score = parseInt($('#braden-scores-form input[name="activity_score"]').val());
	self.updatedRow.mobility_score = parseInt($('#braden-scores-form input[name="mobility_score"]').val());
	self.updatedRow.nutrition_score = parseInt($('#braden-scores-form input[name="nutrition_score"]').val());
	self.updatedRow.friction_shear_score = parseInt($('#braden-scores-form input[name="friction_shear_score"]').val());
	$('#braden-dialog-msg').text('');
    $.post( 'braden-scores-post', $('#braden-scores-form').serialize(), function(response) {
    	console.log("BradenScores posting message: "+response.msg);
    	if (response.msg) {
    		$('#braden-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		if (self.mode == 'Edit') {
    			self.table[self.idx].braden_scoring_date = self.updatedRow.braden_scoring_date;
    			self.table[self.idx].sensory_perception_score = self.updatedRow.sensory_perception_score;
    			self.table[self.idx].moisture_score = self.updatedRow.moisture_score;
    			self.table[self.idx].activity_score = self.updatedRow.activity_score;
    			self.table[self.idx].mobility_score = self.updatedRow.mobility_score;
    			self.table[self.idx].nutrition_score = self.updatedRow.nutrition_score;
    			self.table[self.idx].friction_shear_score = self.updatedRow.friction_shear_score;
    			self.dataTable.fnUpdate(self.updatedRow, self.idx);
    		} else {
    			var row = { id: parseInt(response.id), patient_id: self.patient_id, braden_scoring_date: self.updatedRow.braden_scoring_date,
    					sensory_perception_score: parseInt(self.updatedRow.sensory_perception_score), 
    					moisture_score: parseInt(self.updatedRow.moisture_score),
    					activity_score: parseInt(self.updatedRow.activity_score),
    					mobility_score: parseInt(self.updatedRow.mobility_score),
    					nutrition_score: parseInt(self.updatedRow.nutrition_score),
    					friction_shear_score: parseInt(self.updatedRow.friction_shear_score)};
    			self.table.push(row);
    			self.dataTable.fnAddData(self.updatedRow);
    		}
    	}
    	},
        'json' // I expect a JSON response
    );
}

BradenDialog.prototype.bradenDialogDelete = function () {
	var self = this;
	var d = $(this).data("myData");
	if (!(typeof d === 'undefined')) {
		self.idx = d.idx;
		self.id = d.id;
		self.table = d.table;		
		self.dataTable = d.dataTable;		
	} else {
		return;
	}
	$('#braden-delete-dialog-msg').text('');
    $.post( 'braden-scores-delete', { id: self.id}, function(response) {
    	console.log("BradenScores delete message: "+response.msg);
    	if (response.msg) {
    		$('#braden-delete-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		self.table.splice(self.idx,1);
    		self.dataTable.fnDeleteRow(self.idx);
    	}
    	},
        'json' // I expect a JSON response
    );
}

BradenDialog.prototype.bradenDialogModeNew = function (patient_id, table, dataTable) {
	this.dialog.data("myData", {mode: "New", idx: -1, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Braden scores (New)');
	this.submitLabel = "Create a new set of scores";
	this.dialog.dialog('option', 'buttons', [
   	    {text: this.submitLabel, click : this.bradenDialogSubmit},
  	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
   	    ]);
	$('#braden-scores-form input[name="id"]').val("0");
	$('#braden-scores-form input[name="patient_id"]').val(patient_id);
	$('#braden-dialog-msg').text('');
}

BradenDialog.prototype.bradenDialogModeEdit = function (idx, id, table, dataTable) {
	var rc = 0;
	this.dialog.data("myData", {mode: "Edit", idx: idx, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Braden scores (Edit)');
	this.submitLabel = "Update scores";
	this.dialog.dialog('option', 'buttons', [
	    {text: this.submitLabel, click : this.bradenDialogSubmit},
	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
	    ]);
	// The next JQuery call gets the data for the modal dialog using AJAX
	$('#braden-dialog-msg').text('');
	$.getJSON('braden-scores-single',{id : id},function(response,status,xhr) {
		if (status == 'success') {
			$('#braden-scores-form input[name="id"]').val(response.bradenScores.id);
			$('#braden-scores-form input[name="patient_id"]').val(response.bradenScores.patient_id);
			var d = response.bradenScores.braden_scoring_date;
			$('#braden_scoring_date').datetimepicker({value:d, format:'Y-m-d H:i:s' ,step:30});
			$('#braden-scores-form input[name="sensory_perception_score"]').val(response.bradenScores.sensory_perception_score);
			$('#braden-scores-form input[name="moisture_score"]').val(response.bradenScores.moisture_score);
			$('#braden-scores-form input[name="activity_score"]').val(response.bradenScores.activity_score);
			$('#braden-scores-form input[name="mobility_score"]').val(response.bradenScores.mobility_score);
			$('#braden-scores-form input[name="nutrition_score"]').val(response.bradenScores.nutrition_score);
			$('#braden-scores-form input[name="friction_shear_score"]').val(response.bradenScores.friction_shear_score);
		} else {
			rc = -1;
		}
	});
	return rc;
}

BradenDialog.prototype.bradenDialogModeDelete = function (idx, id, table, dataTable) {
	var rc = 0;
	this.confirmDialog.data("myData", {idx: idx, id: id, table: table, dataTable: dataTable});
	this.confirmDialog.dialog('option', 'buttons', [
	                            			{text: "Yes", click : this.bradenDialogDelete},
	                            			{text: "No", click: function() { $(this).dialog("close"); } }
	                                 	    ]);
	$('#braden-delete-dialog-msg').text('');
	return rc;
}

