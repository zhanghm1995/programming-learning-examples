/**
 * Learning how to use yaml-cpp library to parse and write yaml file
 * 
 */

#include <iostream>
#include <string>
#include <fstream>

#include <yaml-cpp/yaml.h>

using namespace std;

int test_parse_yaml_file()
{
	YAML::Node config;
	try
	{
		config = YAML::LoadFile("../yaml_test.yaml");
	}
	catch (const YAML::Exception &e)
	{
		cout << "Failed load yaml file: " << endl
			 << e.what() << endl;
		return -1;
	}

	cout << config.size() << endl;

	std::string blog_name{"xxxxx"}, blog_id{"xxxxx"}, blog_url{"xxxxx"};
	if (config["blog"]) {
		if (config["blog"]["name"])
			blog_name = config["blog"]["name"].as<std::string>();
		if (config["blog"]["id"])
			blog_id = config["blog"]["id"].as<std::string>();
		if (config["blog"]["url"])
			blog_url = config["blog"]["url"].as<std::string>();
	}
	else {
		fprintf(stderr, "the node blog doesn't exist\n");
	}

	fprintf(stdout, "blog name: %s, id: %s, url: %s\n",
			blog_name.c_str(), blog_id.c_str(), blog_url.c_str());

	bool value1, value2;
	if (config["value1"])
		value1 = config["value1"].as<bool>();
	if (config["value2"])
		value2 = config["value2"].as<bool>();

	fprintf(stdout, "value1: %d, value2: %d\n", value1, value2);

	vector<string> sensors;
	if (config["sensors"]) {
		sensors = config["sensors"]["name"].as<vector<string>>();
	}
	else {
		cout << "no node sensors" << endl;
	}
	for (auto s : sensors) {
		cout << s << endl;
	}

	int number1;
	std::string number2, number3;
	float number4;
	if (config["number1"])
		number1 = config["number1"].as<int>();
	if (config["number2"])
		number2 = config["number2"].as<std::string>();
	if (config["number3"])
		number3 = config["number3"].as<std::string>();
	if (config["number4"])
		number4 = config["number4"].as<float>();
	fprintf(stdout, "number1: %d, number2: %s, number3: %s, number4: %f\n",
			number1, number2.c_str(), number3.c_str(), number4);

	std::string github_url, github_repos;
	if (config["github"])
		github_url = config["github"][0].as<std::string>();
	github_repos = config["github"][1].as<std::string>();
	fprintf(stdout, "github url: %s, repos: %s\n", github_url.c_str(), github_repos.c_str());

	return 0;
}

void test_parse_yaml()
{
	YAML::Node config = YAML::LoadFile("../yaml_test_vector.yaml");

	cout << config.size() << endl;
	cout << config << endl;
	if (config["commands"])
	{
		cout << config["commands"]["version"] << endl;
		cout << config["commands"]["version"].size() << endl;
	}
	else
	{
		cout << "no blog" << endl;
	}
}

int test_generate_yaml_file()
{
	YAML::Node node;

	node["language"] = "cpp";
	node["version"] = 2;

	node["url"].push_back("https://blog.csdn.net/fengbingchun");
	node["url"].push_back("https://github.com/fengbingchun");

	YAML::Node primes = YAML::Load("[2, 3, 5, 7, 11]");
	primes.push_back(13);
	fprintf(stdout, "primes size: %d\n", primes.size());
	node["primes"] = primes;

	YAML::Node lineup = YAML::Load("{1B: Prince Fielder, 2B: Rickie Weeks, LF: Ryan Braun}");
	lineup["RF"] = "Corey Hart";
	lineup["C"] = "Jonathan Lucroy";
	node["name"] = lineup;

	node["platform"]["linux"].push_back("x86");
	node["platform"]["linux"].push_back("x86_64");
	node["platform"]["linux"].push_back("armv7");

	node["platform"]["windows"].push_back("x86");
	node["platform"]["windows"].push_back("x86_64");

	std::ofstream fout("testdata/tmp.yaml");
	fout << node;

	return 0;
}

int main()
{
	test_parse_yaml_file();
	cout << "------------------" << endl;
	test_parse_yaml();

	return 0;
}
