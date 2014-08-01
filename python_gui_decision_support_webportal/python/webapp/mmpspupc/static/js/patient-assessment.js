function AssessmentDialog() {
	this.submitLabel = "Create a new patient assessment";
	this.dialog = $( '#patient-assessment-form-div' ).dialog({
		autoOpen: false,
		height: 750,
		width: 750,
		modal: true,
		buttons: [
			{text: this.submitLabel, click : this.assessmentDialogSubmit()},
			{text: "Cancel", click: function() { $(this).dialog("close"); } }
		],
		close: function() {
			$('form[name="patient-assessment-form"]')[0].reset();
		}
	});
	this.confirmDialog = $( '#patient-assessment-delete-div' ).dialog({
		autoOpen: false,
		height: 150,
		width: 400,
		modal: true,
		close: function() {	$(this).dialog("close"); }
	});
	$('#patient-assessment-form input[name="assessment_date"]').datetimepicker({format:'Y-m-d H:i:s', step:30});
	$('#patient-assessment-form').validate({
		debug: true,
		rules: {
			"assessment_date": {required: true},
			"assessment_note": {required: true},
			"education_notes": {required: false},
			"education_understanding": {required: true},
			"education_evidenced_by": {required: false, maxlength: 45}
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

AssessmentDialog.prototype.assessmentDialogSubmit = function () {
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
	self.patient_id = parseInt($('#patient-assessment-form input[name="patient_id"]').val());
	self.updatedRow = {};
	self.updatedRow.assessment_date = $('#patient-assessment-form input[name="assessment_date"]').val();
	self.updatedRow.assessment_note = $('#patient-assessment-form textarea[name="assessment_note"]').val();
	self.updatedRow.education_notes = $('#patient-assessment-form textarea[name="education_notes"]').val();
	self.updatedRow.education_understanding = $('#patient-assessment-form select[name="education_understanding"]').val();
	self.updatedRow.education_evidenced_by = $('#patient-assessment-form input[name="education_evidenced_by"]').val();
	$('#assessment-dialog-msg').text('');
    $.post( 'patient-assessment-post', $('#patient-assessment-form').serialize(), function(response) {
    	console.log("PatientAssessment posting message: "+response.msg);
    	if (response.msg) {
    		$('#assessment-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		if (self.mode == 'Edit') {
    			self.table[self.idx].assessment_date = self.updatedRow.assessment_date;
    			self.table[self.idx].assessment_note = self.updatedRow.assessment_note;
    			self.table[self.idx].education_notes = self.updatedRow.education_notes;
    			self.table[self.idx].education_understanding = self.updatedRow.education_understanding;
    			self.table[self.idx].education_evidenced_by = self.updatedRow.education_evidenced_by;
    			self.dataTable.fnUpdate(self.updatedRow, self.idx);
    		} else {
    			var row = { id: parseInt(response.id), patient_id: self.patient_id, 
    					assessment_date: self.updatedRow.assessment_date,
    					assessment_note: self.updatedRow.assessment_note,
    					education_notes: self.updatedRow.education_notes,
    					education_understanding: self.updatedRow.education_understanding,
    					education_evidenced_by: self.updatedRow.education_evidenced_by
    					};
    			self.table.push(row);
    			self.dataTable.fnAddData(self.updatedRow);
    		}
    	}
    	},
        'json' // I expect a JSON response
    );
}

AssessmentDialog.prototype.assessmentDialogDelete = function () {
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
	$('#assessment-delete-dialog-msg').text('');
    $.post( 'patient-assessment-delete', { id: self.id}, function(response) {
    	console.log("PatientAssessment delete message: "+response.msg);
    	if (response.msg) {
    		$('#assessment-delete-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		self.table.splice(self.idx,1);
    		self.dataTable.fnDeleteRow(self.idx);
    	}
    	},
        'json' // I expect a JSON response
    );
}

AssessmentDialog.prototype.assessmentDialogModeNew = function (patient_id, table, dataTable) {
	this.dialog.data("myData", {mode: "New", idx: -1, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Patient Assessment (New)');
	this.submitLabel = "Create a new patient assessment";
	this.dialog.dialog('option', 'buttons', [
   	    {text: this.submitLabel, click : this.assessmentDialogSubmit},
  	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
   	    ]);
	$('#patient-assessment-form input[name="id"]').val("0");
	$('#patient-assessment-form input[name="patient_id"]').val(patient_id);
	$('#assessment-dialog-msg').text('');
}

AssessmentDialog.prototype.assessmentDialogModeEdit = function (idx, id, table, dataTable) {
	var rc = 0;
	this.dialog.data("myData", {mode: "Edit", idx: idx, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Patient Assessment (Edit)');
	this.submitLabel = "Update status";
	this.dialog.dialog('option', 'buttons', [
	    {text: this.submitLabel, click : this.assessmentDialogSubmit},
	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
	    ]);
	// The next JQuery call gets the data for the modal dialog using AJAX
	$('#assessment-dialog-msg').text('');
	$.getJSON('patient-assessment-single',{id : id},function(response,status,xhr) {
		if (status == 'success') {
			$('#patient-assessment-form input[name="id"]').val(response.patientAssessment.id);
			$('#patient-assessment-form input[name="patient_id"]').val(response.patientAssessment.patient_id);
			var d = response.patientAssessment.assessment_date;
			$('#patient-assessment-form input[name="assessment_date"]').datetimepicker({value:d, format:'Y-m-d H:i:s' ,step:30});
			$('#patient-assessment-form textarea[name="assessment_note"]').val(response.patientAssessment.assessment_note);
			$('#patient-assessment-form textarea[name="education_notes"]').val(response.patientAssessment.education_notes);
			$('#patient-assessment-form select[name="education_understanding"]').val(response.patientAssessment.education_understanding);
			$('#patient-assessment-form input[name="education_evidenced_by"]').val(response.patientAssessment.education_evidenced_by);
		} else {
			rc = -1;
		}
	});
	return rc;
}

AssessmentDialog.prototype.assessmentDialogModeDelete = function (idx, id, table, dataTable) {
	var rc = 0;
	this.confirmDialog.data("myData", {idx: idx, id: id, table: table, dataTable: dataTable});
	this.confirmDialog.dialog('option', 'buttons', [
	                            			{text: "Yes", click : this.assessmentDialogDelete},
	                            			{text: "No", click: function() { $(this).dialog("close"); } }
	                                 	    ]);
	$('#assessment-delete-dialog-msg').text('');
	return rc;
}

