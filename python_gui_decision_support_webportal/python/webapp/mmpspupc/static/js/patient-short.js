function ShortDialog() {
	this.submitLabel = "Create a new patient record";
	this.dialog = $( '#patient-short-form-div' ).dialog({
		autoOpen: false,
		height: 400,
		width: 750,
		modal: true,
		buttons: [
			{text: this.submitLabel, click : this.shortDialogSubmit()},
			{text: "Cancel", click: function() { $(this).dialog("close"); } }
		],
		close: function() {
			$('form[name="patient-short-form"]')[0].reset();
		}
	});
	$('#patient-short-form').validate({
		debug: true,
		rules: {
			"age": {required: true,	range: [1, 150]},
			"medical_history": {required: true}
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

ShortDialog.prototype.shortDialogSubmit = function () {
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
	self.updatedRow = {};
	self.updatedRow.age = $('#patient-short-form input[name="age"]').val();
	self.updatedRow.medical_history = $('#patient-short-form textarea[name="medical_history"]').val();
	$('#short-dialog-msg').text('');
    $.post( 'patient-short-post', $('#patient-short-form').serialize(), function(response) {
    	console.log("PatientInformation posting message: "+response.msg);
    	if (response.msg) {
    		$('#short-dialog-msg').text(response.msg);
    	} else {
    		$(self).dialog("close");
    		if (self.mode == 'Edit') {
    			self.table[self.idx].age = self.updatedRow.age;
    			self.table[self.idx].medical_history = self.updatedRow.medical_history;
     			self.dataTable.fnUpdate(self.updatedRow, self.idx);
    		}
    	}
    	},
        'json' // I expect a JSON response
    );
}

ShortDialog.prototype.shortDialogModeEdit = function (idx, patient_id, table, dataTable) {
	var rc = 0;
	this.dialog.data("myData", {mode: "Edit", idx: idx, table: table, dataTable: dataTable});
	this.dialog.dialog('option','title', 'Patient information (Edit)');
	this.submitLabel = "Update status";
	this.dialog.dialog('option', 'buttons', [
	    {text: this.submitLabel, click : this.shortDialogSubmit},
	    {text: "Cancel", click: function() { $(this).dialog("close"); } }
	    ]);
	// The next JQuery call gets the data for the modal dialog using AJAX
	$('#short-dialog-msg').text('');
	$.getJSON('patient-short-single',{patient_id : patient_id},function(response,status,xhr) {
		if (status == 'success') {
			$('#patient-short-form input[name="patient_id"]').val(response.patientshort.patient_id);
			$('#patient-short-form input[name="age"]').val(response.patientshort.age);
			$('#patient-short-form textarea[name="medical_history"]').val(response.patientshort.medical_history);
		} else {
			rc = -1;
		}
	});
	return rc;
}
