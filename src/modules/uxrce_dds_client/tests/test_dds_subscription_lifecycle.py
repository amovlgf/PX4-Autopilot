#!/usr/bin/env python3
"""Compile the generated bridge lifecycle against deterministic uORB/XRCE fakes.

Run with python3 -m unittest discover -s src/modules/uxrce_dds_client/tests -v.
The actual generated constructor, init and reset execute; the mocks track owned
handles (including valid handle zero) and inject subscription/writer failures.
"""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1]
TEMPLATE = MODULE / "dds_topics.h.em"

PRELUDE = r"""
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << "line " << __LINE__ << ": " #condition << '\n'; std::abort(); \
} } while (false)
constexpr int ORB_SUB_INVALID = -1;
constexpr int POLLIN = 1;
constexpr int UXR_INVALID_ID = 0;
constexpr int UXRCE_DEFAULT_POLL_INTERVAL_MS = 4;
struct px4_pollfd_struct_t { int fd; short events; short revents; };
struct ucdrBuffer {};
struct uxrSession {};
using uxrStreamId = int;
struct uxrObjectId { int id; int type; };
uxrObjectId uxr_object_id(int id, int type) { return {id, type}; }
enum class ORB_ID { alpha, beta, gamma };
struct orb_metadata { int o_id; };
#define ORB_ID(name) (&metadata_##name)
static std::set<int> open_handles;
static std::vector<int> unsubscribed;
static std::vector<int> intervals;
static int next_handle = 10;
static int subscription_calls = 0;
static int writer_calls = 0;
static int fail_subscription_at = -1;
static int fail_writer_at = -1;
bool orb_sub_valid(int fd) { return fd >= 0; }
int orb_subscribe_multi(const orb_metadata *, uint8_t)
{
    if (subscription_calls++ == fail_subscription_at) { return ORB_SUB_INVALID; }
    while (open_handles.count(next_handle)) { ++next_handle; }
    const int fd = next_handle++;
    CHECK(open_handles.insert(fd).second);
    return fd;
}
int orb_unsubscribe(int fd)
{
    unsubscribed.push_back(fd);
    CHECK(open_handles.erase(fd) == 1);
    return 0;
}
int orb_set_interval(int fd, uint64_t)
{
    CHECK(open_handles.count(fd));
    intervals.push_back(fd);
    return 0;
}
bool create_data_writer(uxrSession *, uxrStreamId, uxrObjectId, ORB_ID,
                        const char *, const char *, uint32_t, uint8_t,
                        const char *, uxrObjectId &writer)
{
    if (writer_calls++ == fail_writer_at) { return false; }
    writer = {writer_calls, 1};
    return true;
}
"""

SCENARIOS = r"""
static bool connect(SendTopicsSubs &subs)
{
    return subs.init(nullptr, 0, 0, {1, 1}, nullptr);
}
static void check_reset(const SendTopicsSubs &subs)
{
    CHECK(subs.num_payload_sent == 0);
    for (const auto &fd : subs.fds) {
        CHECK(fd.fd == ORB_SUB_INVALID);
        CHECK(fd.events == 0);
        CHECK(fd.revents == 0);
    }
    for (const auto &sub : subs.send_subscriptions) {
        CHECK(sub.data_writer.type == UXR_INVALID_ID);
    }
}
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    const std::string scenario(argv[1]);
    SendTopicsSubs subs;
    if (scenario == "reset_before_init") {
        // Handle zero belongs to dataman, not to this DDS bridge.
        open_handles.insert(0);
        subs.reset();
        subs.reset();
        CHECK(unsubscribed.empty());
        CHECK(open_handles == std::set<int>{0});
        check_reset(subs);
    } else if (scenario == "subscription_failure" || scenario == "partial_subscription_failure") {
        open_handles.insert(0);
        fail_subscription_at = scenario == "subscription_failure" ? 0 : 1;
        CHECK(!connect(subs));
        CHECK(subs.fds[fail_subscription_at].events == 0);
        CHECK(subs.fds[fail_subscription_at].fd == ORB_SUB_INVALID);
        CHECK(writer_calls == fail_subscription_at);
        CHECK(intervals.size() == static_cast<unsigned>(fail_subscription_at));
        subs.reset();
        CHECK(unsubscribed.size() == static_cast<unsigned>(fail_subscription_at));
        CHECK(open_handles == std::set<int>{0});
        check_reset(subs);
        fail_subscription_at = -1;
        CHECK(connect(subs));
        CHECK(open_handles.size() == 4);
        subs.reset();
        CHECK(open_handles == std::set<int>{0});
    } else if (scenario == "partial_writer_failure") {
        open_handles.insert(0);
        fail_writer_at = 1;
        CHECK(!connect(subs));
        CHECK(subscription_calls == 2);
        subs.reset();
        CHECK(unsubscribed.size() == 2);
        CHECK(open_handles == std::set<int>{0});
        check_reset(subs);
        fail_writer_at = -1;
        CHECK(connect(subs));
        subs.reset();
        CHECK(unsubscribed.size() == 5);
        CHECK(open_handles == std::set<int>{0});
    } else if (scenario == "reset_reconnect") {
        open_handles.insert(0);
        for (int cycle = 0; cycle < 3; ++cycle) {
            CHECK(connect(subs));
            CHECK(subscription_calls == (cycle + 1) * 3);
            CHECK(open_handles.size() == 4);
            // Calling init again must not leak subscriptions.
            CHECK(connect(subs));
            CHECK(subscription_calls == (cycle + 1) * 3);
            subs.num_payload_sent = 42;
            subs.fds[0].revents = POLLIN;
            subs.reset();
            subs.reset();
            CHECK(unsubscribed.size() == static_cast<unsigned>((cycle + 1) * 3));
            CHECK(open_handles == std::set<int>{0});
            check_reset(subs);
        }
    } else if (scenario == "owned_zero") {
        next_handle = 0;
        CHECK(connect(subs));
        CHECK(subs.fds[0].fd == 0);
        subs.reset();
        CHECK(unsubscribed == (std::vector<int>{0, 1, 2}));
        CHECK(open_handles.empty());
        subs.reset();
        CHECK(unsubscribed.size() == 3);
        check_reset(subs);
    } else { CHECK(false); }
}
"""


class SubscriptionLifecycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++")
        if not compiler:
            raise RuntimeError("A C++ compiler is required for the DDS lifecycle regression")
        cls.temporary = tempfile.TemporaryDirectory(prefix="px4-dds-lifecycle-")
        cls.addClassCleanup(cls.temporary.cleanup)
        root = Path(cls.temporary.name)
        topics = root / "topics.yaml"
        topics.write_text(json.dumps({"publications": [
            {"topic": "/fmu/out/" + name, "type": "px4_msgs::msg::" + name.title()}
            for name in ("alpha", "beta", "gamma")
        ]}))
        subprocess.run([
            sys.executable, str(MODULE / "generate_dds_topics.py"),
            "--dds-topics-file", str(topics), "--template_file", str(TEMPLATE),
            "--client-outdir", str(root),
        ], check=True, capture_output=True, text=True)
        generated = (root / TEMPLATE.name.replace(".em", "")).read_text()
        # Keep the production declarations, constructor, init and reset verbatim;
        # update/RX serialization are outside the lifecycle under test.
        begin = generated.index("typedef bool (*UcdrSerializeMethod)")
        end = generated.index("void SendTopicsSubs::update(")
        topic_stubs = ""
        for index, name in enumerate(("alpha", "beta", "gamma")):
            topic_stubs += """
static orb_metadata metadata_%(name)s{%(index)d};
struct %(name)s_s { uint64_t timestamp; };
uint32_t ucdr_topic_size_%(name)s() { return 8; }
bool ucdr_serialize_%(name)s(const void *, ucdrBuffer &, int64_t) { return true; }
""" % {"name": name, "index": index}
        source = root / "lifecycle.cpp"
        source.write_text(PRELUDE + topic_stubs + generated[begin:end] + SCENARIOS)
        cls.binary = root / "lifecycle"
        compiled = subprocess.run([
            compiler, "-std=c++14", "-Wall", "-Wextra", "-Werror",
            "-Wno-unused-parameter", str(source), "-o", str(cls.binary),
        ], capture_output=True, text=True)
        if compiled.returncode:
            raise RuntimeError(compiled.stdout + compiled.stderr)

    def run_scenario(self, name):
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_reset_before_init_does_not_close_dataman(self):
        self.run_scenario("reset_before_init")

    def test_subscription_failure_does_not_create_writer_or_poll_invalid_handle(self):
        self.run_scenario("subscription_failure")

    def test_partial_subscription_failure_cleans_only_owned_handles(self):
        self.run_scenario("partial_subscription_failure")

    def test_partial_writer_failure_can_cleanup_and_reconnect(self):
        self.run_scenario("partial_writer_failure")

    def test_repeated_reset_and_reconnect_preserve_other_subscriptions(self):
        self.run_scenario("reset_reconnect")

    def test_owned_handle_zero_is_closed(self):
        self.run_scenario("owned_zero")


if __name__ == "__main__":
    unittest.main()
