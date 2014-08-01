function TreatmentDialog() {
	this.submitLabel = "Create a new treatment plan";
	this.dialog = $( '#treatment-plan-form-div' ).dialog({
		autoOpen: false,
		height: 550,
		width: 750,
		modal: true,
		buttons: [
			{text: this.submitLabel, click : this.treatmentDialogSubmit()},
			{text: "Cancel", click: function() { $(this).dialog("close"); } }
		],
		close: function() {
			$('form[name="treatment-plan-form"]')[0].reset();
		}
	});
	this.confirmDialog = $( '#treatment-plan-delete-div' ).dialog({
		autoOpen: false,
		height: 150,
		width: 400,
		modal: true,
		close: function() {	$(this).dialog("close"); }
	});
	$('#treatment-plan-form input[name="plan_date"]').datetimepicker({format:'Y-m-d H:i:s', step:30});
	$('#treatment-plan-form').validate({
		debug: true,
		rules: {
			"plan_date": {required: true},
			"plan_notes": {required: true}
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

TreatmentDialog.prototype.treatmentDialogSubmit = function () {
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
	self.patient_id = parseInt($('#treatment-plan-form input[name="patient_id"]').val());
	self.updatedRow = {};
	self.updatedRow.plan_date = $('#treatment-plan-form input[name="plan_date"]').val();
	self.updatedRow.plan_notes = $('#treatment-plan-form textarea[name="plan_notes"]').val();
	$('#treatment-dialog-msg').text('');
    $.post( 'treatment-plan-post', $('#treatment-plan-form').serialize(), function(response) {
    	console.log("TreatmentPlan posting message: "+response.msg);
    	if (response.msg) {
    		$('#treatment-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		if (self.mode == 'Edit') {
    			self.table[self.idx].plan_date = self.updatedRow.plan_date;
    			self.table[self.idx].plan_notes = self.updatedRow.plan_notes;
    			self.dataTable.fnUpdate(self.updatedRow, self.idx);
    		} else {
    			var row = { id: parseInt(response.id), patient_id: self.patient_id, 
    					plan_date: self.updatedRow.plan_date,
    					plan_notes: self.updatedRow.plan_notes
    					};
    			self.table.push(row);
    			self.dataTable.fnAddData(self.updatedRow);
    		}
    	}
    	},
        'json' // I expect a JSON response
    );
}

TreatmentDialog.prototype.treatmentDialogDelete = function () {
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
	$('#treatment-delete-dialog-msg').text('');
    $.post( 'treatment-plan-delete', { id: self.id}, function(response) {
    	console.log("TreatmentPlan delete message: "+response.msg);
    	if (response.msg) {
    		$('#treatment-delete-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		self.table.splice(self.idx,1);
    		self.dataTable.fnDeleteRow(self.idx);
    	}
    	},
        'json' // I expect a JSON response
    );
}

TreatmentDialog.prototype.treatmentDialogModeNew = function (patient_id, table, dataTable) {
	this.dialog.data("myData", {mode: "New", idx: -1, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Treatment plan (New)');
	this.submitLabel = "Create a new treatment plan";
	this.dialog.dialog('option', 'buttons', [
   	    {text: this.submitLabel, click : this.treatmentDialogSubmit},
  	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
   	    ]);
	$('#treatment-plan-form input[name="id"]').val("0");
	$('#treatment-plan-form input[name="patient_id"]').val(patient_id);
	$('#treatment-dialog-msg').text('');
}

TreatmentDialog.prototype.treatmentDialogModeEdit = function (idx, id, table, dataTable) {
	var rc = 0;
	this.dialog.data("myData", {mode: "Edit", idx: idx, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Treatment plan (Edit)');
	this.submitLabel = "Update status";
	this.dialog.dialog('option', 'buttons', [
	    {text: this.submitLabel, click : this.treatmentDialogSubmit},
	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
	    ]);
	// The next JQuery call gets the data for the modal dialog using AJAX
	$('#treatment-dialog-msg').text('');
	$.getJSON('treatment-plan-single',{id : id},function(response,status,xhr) {
		if (status == 'success') {
			$('#treatment-plan-form input[name="id"]').val(response.treatmentPlan.id);
			$('#treatment-plan-form input[name="patient_id"]').val(response.treatmentPlan.patient_id);
			var d = response.treatmentPlan.plan_date;
			$('#treatment-plan-form input[name="plan_date"]').datetimepicker({value:d, format:'Y-m-d H:i:s' ,step:30});
			$('#treatment-plan-form textarea[name="plan_notes"]').val(response.treatmentPlan.plan_notes);
		} else {
			rc = -1;
		}
	});
	return rc;
}

TreatmentDialog.prototype.treatmentDialogModeDelete = function (idx, id, table, dataTable) {
	var rc = 0;
	this.confirmDialog.data("myData", {idx: idx, id: id, table: table, dataTable: dataTable});
	this.confirmDialog.dialog('option', 'buttons', [
	                            			{text: "Yes", click : this.treatmentDialogDelete},
	                            			{text: "No", click: function() { $(this).dialog("close"); } }
	                                 	    ]);
	$('#treatment-delete-dialog-msg').text('');
	return rc;
}

