static const struct of_device_id aht10_of_match[] = {
	{ .compatible = "aosong,aht10", },
	{ },
};
MODULE_DEVICE_TABLE(of, aht10_of_match);
.of_match_table = of_match_ptr(aht10_of_match), // (in .driver for i2c_driver)
