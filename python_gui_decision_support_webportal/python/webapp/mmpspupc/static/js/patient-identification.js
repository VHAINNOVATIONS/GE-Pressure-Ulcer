function PatientDialog() {
	this.submitLabel = "Create a new patient record";
	this.dialog = $( '#patient-identification-form-div' ).dialog({
		autoOpen: false,
		height: 700,
		width: 750,
		modal: true,
		buttons: [
			{text: this.submitLabel, click : this.patientDialogSubmit()},
			{text: "Cancel", click: function() { $(this).dialog("close"); } }
		],
		close: function() {
			$('form[name="patient-identification-form"]')[0].reset();
		}
	});
	this.confirmDialog = $( '#patient-identification-delete-div' ).dialog({
		autoOpen: false,
		height: 150,
		width: 400,
		modal: true,
		close: function() {	$(this).dialog("close"); }
	});
	$('#patient-identification-form').validate({
		debug: true,
		rules: {
			"va_patient_id": {required: true},
			"patient_name": {required: true},
			"age": {required: true,	range: [1, 150]},
			"medical_history": {required: true},
			"camera_id": {required: true,	range: [1, 99999]}
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

PatientDialog.prototype.patientDialogSubmit = function () {
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
	self.patient_id = parseInt($('#patient-identification-form input[name="patient_id"]').val());
	self.updatedRow = {};
	self.updatedRow.va_patient_id = $('#patient-identification-form input[name="va_patient_id"]').val();
	self.updatedRow.patient_name = $('#patient-identification-form input[name="patient_name"]').val();
	self.updatedRow.age = parseInt($('#patient-identification-form input[name="age"]').val());
	self.medical_history = $('#patient-identification-form textarea[name="medical_history"]').val();
	self.camera_id = parseInt($('#patient-identification-form input[name="camera_id"]').val());
	$('#patient-dialog-msg').text('');
    $.post( 'patient-identification-post', $('#patient-identification-form').serialize(), function(response) {
    	console.log("PatientIdentification posting message: "+response.msg);
    	if (response.msg) {
    		$('#patient-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		if (self.mode == 'Edit') {
    			self.table[self.idx].va_patient_id = self.updatedRow.va_patient_id;
    			self.table[self.idx].patient_name = self.updatedRow.patient_name;
    			self.table[self.idx].age = self.updatedRow.age;
    			self.table[self.idx].medical_history = self.medical_history;
    			self.table[self.idx].camera_id = self.camera_id;
     			self.dataTable.fnUpdate(self.updatedRow, self.idx);
    		} else {
    			var row = { patient_id: parseInt(response.patient_id), 
    					va_patient_id: self.updatedRow.va_patient_id,
    					patient_name: self.updatedRow.patient_name,
    					age: self.updatedRow.age,
    					medical_history: self.medical_history,
    					camera_id: self.camera_id
    					};
    			self.table.push(row);
    			self.dataTable.fnAddData(self.updatedRow);
    		}
    	}
    	},
        'json' // I expect a JSON response
    );
}

PatientDialog.prototype.patientDialogDelete = function () {
	var self = this;
	var d = $(this).data("myData");
	if (!(typeof d === 'undefined')) {
		self.idx = d.idx;
		self.patient_id = d.patient_id;
		self.table = d.table;		
		self.dataTable = d.dataTable;		
	} else {
		return;
	}
	$('#patient-delete-dialog-msg').text('');
    $.post( 'patient-identification-delete', { patient_id: self.patient_id}, function(response) {
    	console.log("PatientAdmission delete message: "+response.msg);
    	if (response.msg) {
    		$('#patient-delete-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		self.table.splice(self.idx,1);
    		self.dataTable.fnDeleteRow(self.idx);
    	}
    	},
        'json' // I expect a JSON response
    );
}

PatientDialog.prototype.patientDialogModeNew = function (patient_id, table, dataTable) {
	this.dialog.data("myData", {mode: "New", idx: -1, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Patient (New)');
	this.submitLabel = "Create a new patient";
	this.dialog.dialog('option', 'buttons', [
   	    {text: this.submitLabel, click : this.patientDialogSubmit},
  	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
   	    ]);
	$('#patient-identification-form input[name="patient_id"]').val("0");
	$('#identification-dialog-msg').text('');
}

PatientDialog.prototype.patientDialogModeEdit = function (idx, patient_id, table, dataTable) {
	var rc = 0;
	this.dialog.data("myData", {mode: "Edit", idx: idx, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Patient (Edit)');
	this.submitLabel = "Update status";
	this.dialog.dialog('option', 'buttons', [
	    {text: this.submitLabel, click : this.patientDialogSubmit},
	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
	    ]);
	// The next JQuery call gets the data for the modal dialog using AJAX
	$('#patient-dialog-msg').text('');
	$.getJSON('patientidentification',{patient_id : patient_id},function(response,status,xhr) {
		if (status == 'success') {
			$('#patient-identification-form input[name="patient_id"]').val(response.patient.patient_id);
			$('#patient-identification-form input[name="va_patient_id"]').val(response.patient.va_patient_id);
			$('#patient-identification-form input[name="patient_name"]').val(response.patient.patient_name);
			$('#patient-identification-form input[name="age"]').val(response.patient.age);
			$('#patient-identification-form textarea[name="medical_history"]').val(response.patient.medical_history);
			$('#patient-identification-form input[name="camera_id"]').val(response.patient.camera_id);
		} else {
			rc = -1;
		}
	});
	return rc;
}

PatientDialog.prototype.patientDialogModeDelete = function (idx, patient_id, table, dataTable) {
	var rc = 0;
	this.confirmDialog.data("myData", {idx: idx, patient_id: patient_id, table: table, dataTable: dataTable});
	this.confirmDialog.dialog('option', 'buttons', [
	                            			{text: "Yes", click : this.patientDialogDelete},
	                            			{text: "No", click: function() { $(this).dialog("close"); } }
	                                 	    ]);
	$('#patient-delete-dialog-msg').text('');
	return rc;
}

