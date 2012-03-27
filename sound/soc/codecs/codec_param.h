/*
** ===============================
**	Audio codec WM8903 gain configuration
** ===============================
*/

enum project_id {
	TF101 = 0,      //TF101(EP101)
	SL101,           //SL101(EP102)
};

struct wm8903_parameters{
	u8 analog_speaker_volume;
	u8 analog_headset_volume;
	u8 analog_DMIC_ADC_volume;
	u8 analog_headset_mic_volume;
};


