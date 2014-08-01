function NutritionalDialog() {
	this.submitLabel = "Create a new nutritional status";
	this.dialog = $( '#nutritional-status-form-div' ).dialog({
		autoOpen: false,
		height: 550,
		width: 750,
		modal: true,
		buttons: [
			{text: this.submitLabel, click : this.nutritionalDialogSubmit()},
			{text: "Cancel", click: function() { $(this).dialog("close"); } }
		],
		close: function() {
			$('form[name="nutritional-status-form"]')[0].reset();
		}
	});
	this.confirmDialog = $( '#nutritional-status-delete-div' ).dialog({
		autoOpen: false,
		height: 150,
		width: 400,
		modal: true,
		close: function() {	$(this).dialog("close"); }
	});
	$('#nutritional-status-form input[name="assessment_date"]').datetimepicker({format:'Y-m-d H:i:s', step:30});
	$('#nutritional-status-form').validate({
		debug: true,
		rules: {
			"assessment_date": {required: true},
			"nutritional_notes": {required: true}
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

NutritionalDialog.prototype.nutritionalDialogSubmit = function () {
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
	self.patient_id = parseInt($('#nutritional-status-form input[name="patient_id"]').val());
	self.updatedRow = {};
	self.updatedRow.assessment_date = $('#nutritional-status-form input[name="assessment_date"]').val();
	self.updatedRow.nutritional_notes = $('#nutritional-status-form textarea[name="nutritional_notes"]').val();
	$('#nutritional-dialog-msg').text('');
    $.post( 'nutritional-status-post', $('#nutritional-status-form').serialize(), function(response) {
    	console.log("NutritionalStatus posting message: "+response.msg);
    	if (response.msg) {
    		$('#nutritional-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		if (self.mode == 'Edit') {
    			self.table[self.idx].assessment_date = self.updatedRow.assessment_date;
    			self.table[self.idx].nutritional_notes = self.updatedRow.nutritional_notes;
    			self.dataTable.fnUpdate(self.updatedRow, self.idx);
    		} else {
    			var row = { id: parseInt(response.id), patient_id: self.patient_id, 
    					assessment_date: self.updatedRow.assessment_date,
    					nutritional_notes: self.updatedRow.nutritional_notes
    					};
    			self.table.push(row);
    			self.dataTable.fnAddData(self.updatedRow);
    		}
    	}
    	},
        'json' // I expect a JSON response
    );
}

NutritionalDialog.prototype.nutritionalDialogDelete = function () {
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
	$('#nutritional-delete-dialog-msg').text('');
    $.post( 'nutritional-status-delete', { id: self.id}, function(response) {
    	console.log("NutritionalStatus delete message: "+response.msg);
    	if (response.msg) {
    		$('#nutritional-delete-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		self.table.splice(self.idx,1);
    		self.dataTable.fnDeleteRow(self.idx);
    	}
    	},
        'json' // I expect a JSON response
    );
}

NutritionalDialog.prototype.nutritionalDialogModeNew = function (patient_id, table, dataTable) {
	this.dialog.data("myData", {mode: "New", idx: -1, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Nutritional status (New)');
	this.submitLabel = "Create a new nutritional status";
	this.dialog.dialog('option', 'buttons', [
   	    {text: this.submitLabel, click : this.nutritionalDialogSubmit},
  	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
   	    ]);
	$('#nutritional-status-form input[name="id"]').val("0");
	$('#nutritional-status-form input[name="patient_id"]').val(patient_id);
	$('#nutritional-dialog-msg').text('');
}

NutritionalDialog.prototype.nutritionalDialogModeEdit = function (idx, id, table, dataTable) {
	var rc = 0;
	this.dialog.data("myData", {mode: "Edit", idx: idx, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Nutritional status (Edit)');
	this.submitLabel = "Update status";
	this.dialog.dialog('option', 'buttons', [
	    {text: this.submitLabel, click : this.nutritionalDialogSubmit},
	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
	    ]);
	// The next JQuery call gets the data for the modal dialog using AJAX
	$('#nutritional-dialog-msg').text('');
	$.getJSON('nutritional-status-single',{id : id},function(response,status,xhr) {
		if (status == 'success') {
			$('#nutritional-status-form input[name="id"]').val(response.nutritionalStatus.id);
			$('#nutritional-status-form input[name="patient_id"]').val(response.nutritionalStatus.patient_id);
			var d = response.nutritionalStatus.assessment_date;
			$('#nutritional-status-form input[name="assessment_date"]').datetimepicker({value:d, format:'Y-m-d H:i:s' ,step:30});
			$('#nutritional-status-form textarea[name="nutritional_notes"]').val(response.nutritionalStatus.nutritional_notes);
		} else {
			rc = -1;
		}
	});
	return rc;
}

NutritionalDialog.prototype.nutritionalDialogModeDelete = function (idx, id, table, dataTable) {
	var rc = 0;
	this.confirmDialog.data("myData", {idx: idx, id: id, table: table, dataTable: dataTable});
	this.confirmDialog.dialog('option', 'buttons', [
	                            			{text: "Yes", click : this.nutritionalDialogDelete},
	                            			{text: "No", click: function() { $(this).dialog("close"); } }
	                                 	    ]);
	$('#nutritional-delete-dialog-msg').text('');
	return rc;
}

