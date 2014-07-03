var _sliderInterval;
var _startSliderInterval;
jQuery(document).ready(function(){
	/* This code is executed after the DOM has been completely loaded */
		
	jQuery('#slides .slide').css({position:'absolute', top:0, left:0, display:'none'}).eq(0).css('display','block').attr('data-selectedslide', 'true');

	jQuery('#menu ul li a').click(function(e,keepScroll){

			/* On a thumbnail click */

			jQuery('li.menuItem').removeClass('act').addClass('inact');
			jQuery(this).parent().addClass('act');
			
			var pos = jQuery(this).parent().prevAll('.menuItem').length;
			
			// fade out previous slide
			jQuery('div[data-selectedslide="true"]').attr('data-selectedslide', null).fadeOut(450);
			
			e.preventDefault();
			///* Prevent the default action of the link */
			var anchor = jQuery('a[name="'+jQuery(this).attr('href').substr(1)+'"]');
			var slide = anchor.next();
			slide.detach();
			anchor.after(slide);
			slide.attr('data-selectedslide','true').fadeIn(450);
			slide.children('p').focus();
			
			// Stop the auto-advance if an icon has been clicked:
			if(!keepScroll)  {
				clearInterval(_sliderInterval);
				_sliderInterval = null;
			}
			
			
	});
	
	jQuery('#menu ul li.menuItem:first').addClass('act').siblings().addClass('inact');
	/* On page load, mark the first thumbnail as active */
	
	
	
	/*****
	 *
	 *	Enabling auto-advance.
	 *
	 ****/
	 
	var current=1;
	function autoAdvance()
	{
		if(current==-1) return false;
		
		jQuery('#menu ul li a').eq(current%jQuery('#menu ul li a').length).trigger('click',[true]);	// [true] will be passed as the keepScroll parameter of the click function on line 28
		current++;
	}

	// The number of seconds that the slider will auto-advance in:
	
	var changeEvery = 10;
	
	_startSliderInterval = function() {
		_sliderInterval = setInterval(autoAdvance,changeEvery*1000);
	};
	_startSliderInterval();

	/* End of customizations */
});

function stopMainSlider() {
	if (_sliderInterval) {
		clearInterval(_sliderInterval);
		_sliderInterval = null;
	}
};

function startMainSlider() {
	if (!_sliderInterval) {
		_startSliderInterval();
	}
};