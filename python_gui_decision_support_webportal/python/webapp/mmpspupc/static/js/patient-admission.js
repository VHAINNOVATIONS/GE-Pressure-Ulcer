function AdmissionDialog() {
	this.submitLabel = "Create a new patient admission record";
	this.dialog = $( '#patient-admission-form-div' ).dialog({
		autoOpen: false,
		height: 700,
		width: 750,
		modal: true,
		buttons: [
			{text: this.submitLabel, click : this.admissionDialogSubmit()},
			{text: "Cancel", click: function() { $(this).dialog("close"); } }
		],
		close: function() {
			$('form[name="patient-admission-form"]')[0].reset();
		}
	});
	this.confirmDialog = $( '#patient-admission-delete-div' ).dialog({
		autoOpen: false,
		height: 150,
		width: 400,
		modal: true,
		close: function() {	$(this).dialog("close"); }
	});
	$('#patient-admission-form input[name="admission_date"]').datetimepicker({format:'Y-m-d H:i:s', step:30});
	$('#patient-admission-form').validate({
		debug: true,
		rules: {
			"admission_date": {required: true},
			"admission_note": {required: true},
			"factors_impairing_healing": {required: false, maxlength: 512},
			"patient_group": {required: true}
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

AdmissionDialog.prototype.admissionDialogSubmit = function () {
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
	self.patient_id = parseInt($('#patient-admission-form input[name="patient_id"]').val());
	self.updatedRow = {};
	self.updatedRow.admission_date = $('#patient-admission-form input[name="admission_date"]').val();
	self.updatedRow.admission_note = $('#patient-admission-form textarea[name="admission_note"]').val();
	self.updatedRow.factors_impairing_healing = $('#patient-admission-form textarea[name="factors_impairing_healing"]').val();
	self.updatedRow.patient_group = $('#patient-admission-form select[name="patient_group"]').val();
	$('#admission-dialog-msg').text('');
    $.post( 'patient-admission-post', $('#patient-admission-form').serialize(), function(response) {
    	console.log("PatientAdmission posting message: "+response.msg);
    	if (response.msg) {
    		$('#admission-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		if (self.mode == 'Edit') {
    			self.table[self.idx].admission_date = self.updatedRow.admission_date;
    			self.table[self.idx].admission_note = self.updatedRow.admission_note;
    			self.table[self.idx].factors_impairing_healing = self.updatedRow.factors_impairing_healing;
    			self.table[self.idx].patient_group = self.updatedRow.patient_group;
     			self.dataTable.fnUpdate(self.updatedRow, self.idx);
    		} else {
    			var row = { id: parseInt(response.id), patient_id: self.patient_id, 
    					admission_date: self.updatedRow.admission_date,
    					admission_note: self.updatedRow.admission_note,
    					factors_impairing_healing: self.updatedRow.factors_impairing_healing,
    					patient_group: self.updatedRow.patient_group
    					};
    			self.table.push(row);
    			self.dataTable.fnAddData(self.updatedRow);
    		}
    	}
    	},
        'json' // I expect a JSON response
    );
}

AdmissionDialog.prototype.admissionDialogDelete = function () {
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
	$('#admission-delete-dialog-msg').text('');
    $.post( 'patient-admission-delete', { id: self.id}, function(response) {
    	console.log("PatientAdmission delete message: "+response.msg);
    	if (response.msg) {
    		$('#admission-delete-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		self.table.splice(self.idx,1);
    		self.dataTable.fnDeleteRow(self.idx);
    	}
    	},
        'json' // I expect a JSON response
    );
}

AdmissionDialog.prototype.admissionDialogModeNew = function (patient_id, table, dataTable) {
	this.dialog.data("myData", {mode: "New", idx: -1, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Patient Admission (New)');
	this.submitLabel = "Create a new patient admission";
	this.dialog.dialog('option', 'buttons', [
   	    {text: this.submitLabel, click : this.admissionDialogSubmit},
  	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
   	    ]);
	$('#patient-admission-form input[name="id"]').val("0");
	$('#patient-admission-form input[name="patient_id"]').val(patient_id);
	$('#admission-dialog-msg').text('');
}

AdmissionDialog.prototype.admissionDialogModeEdit = function (idx, id, table, dataTable) {
	var rc = 0;
	this.dialog.data("myData", {mode: "Edit", idx: idx, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Patient Admission (Edit)');
	this.submitLabel = "Update status";
	this.dialog.dialog('option', 'buttons', [
	    {text: this.submitLabel, click : this.admissionDialogSubmit},
	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
	    ]);
	// The next JQuery call gets the data for the modal dialog using AJAX
	$('#admission-dialog-msg').text('');
	$.getJSON('patient-admission-single',{id : id},function(response,status,xhr) {
		if (status == 'success') {
			$('#patient-admission-form input[name="id"]').val(response.patientAdmission.id);
			$('#patient-admission-form input[name="patient_id"]').val(response.patientAdmission.patient_id);
			var d = response.patientAdmission.admission_date;
			$('#patient-admission-form input[name="admission_date"]').datetimepicker({value:d, format:'Y-m-d H:i:s' ,step:30});
			$('#patient-admission-form textarea[name="admission_note"]').val(response.patientAdmission.admission_note);
			$('#patient-admission-form textarea[name="factors_impairing_healing"]').val(response.patientAdmission.factors_impairing_healing);
			$('#patient-admission-form select[name="patient_group"]').val(response.patientAdmission.patient_group);
		} else {
			rc = -1;
		}
	});
	return rc;
}

AdmissionDialog.prototype.admissionDialogModeDelete = function (idx, id, table, dataTable) {
	var rc = 0;
	this.confirmDialog.data("myData", {idx: idx, id: id, table: table, dataTable: dataTable});
	this.confirmDialog.dialog('option', 'buttons', [
	                            			{text: "Yes", click : this.admissionDialogDelete},
	                            			{text: "No", click: function() { $(this).dialog("close"); } }
	                                 	    ]);
	$('#admission-delete-dialog-msg').text('');
	return rc;
}

