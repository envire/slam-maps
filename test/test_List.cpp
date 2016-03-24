#include <boost/test/unit_test.hpp>
#include <maps/grid/List.hpp>
#include <maps/grid/SurfacePatches.hpp>

#include <chrono>

using namespace ::maps;

class Element : public boost::intrusive::list_base_hook<auto_unlink_mode>
{
    int val;
public:
    operator int() const {return val; }
    int value() const { return val; }
    Element& operator=(const int& newVal) { val = newVal; return *this;}
    Element(const int& val_ = int()) : val(val_) {}
    bool operator==(const int& oth) const {return val == oth; }

};


BOOST_AUTO_TEST_CASE(test_list_basic)
{
    List<Element> list;
    BOOST_CHECK_EQUAL(list.size(), 0);

    List<Element> list2 = list;
    BOOST_CHECK_EQUAL(list2.size(), 0);

    list.insertHead(1);
    list.insertTail(2);
    list.insertHead(0);
    List<Element>::const_iterator it=list.begin();
    for(int i=0; i<3; ++i)
    {
        BOOST_CHECK_EQUAL(it->value(), i);
        ++it;
    }
    BOOST_CHECK(it == list.end());

    list2 = list;
    BOOST_CHECK_EQUAL(list2.size(), 3);
    List<Element>::const_iterator it1 = list.begin(), it2=list2.begin();
    BOOST_CHECK(it1 != it2);
    for(int i=0; i<3; ++i)
    {
        BOOST_CHECK_EQUAL(it1->value(), it2->value());
    }

    list2.clear();
    BOOST_CHECK_EQUAL(list2.size(), 0);
}

struct LeakTest : public boost::intrusive::list_base_hook<auto_unlink_mode>
{
    int val;
    static size_t count;
    LeakTest(int val_ = int()) : val(val_)
    {
        if(count > 200) throw std::bad_alloc(); // simulate out of memory exception
        ++count;
    }
    LeakTest(const LeakTest& oth) : list_base_hook(oth), val(oth.val) {++count;}
    ~LeakTest() {BOOST_ASSERT(count > 0); --count;}
};

size_t LeakTest::count = 0;

BOOST_AUTO_TEST_CASE(test_list_memleak)
{
    typedef List<LeakTest> LeakList;
    BOOST_CHECK_EQUAL(LeakTest::count, 0);
    {
        LeakList list;
        for(size_t i=0; i<50; ++i)
        {
            BOOST_CHECK_EQUAL(LeakTest::count, i);
            list.insertHead(LeakTest(i));
        }
        for(size_t i=0; i<25; ++i )
        {
            BOOST_CHECK_EQUAL(LeakTest::count, 50-i);
            LeakList::iterator it = list.erase(list.begin());
            BOOST_CHECK(it == list.begin());
        }
        list.clear();
        BOOST_CHECK_EQUAL(LeakTest::count, 0);
        try
        {
            for(size_t i=0; i<300; ++i)
            {
                BOOST_CHECK_EQUAL(LeakTest::count, i);
                list.insertTail(LeakTest(i));
            }
        } catch (const std::bad_alloc&)
        {
            BOOST_CHECK_EQUAL(LeakTest::count, list.size());
        }
        BOOST_CHECK_EQUAL(LeakTest::count, list.size());
    }
    // after the list runs out of scope, all elements must be deleted again:
    BOOST_CHECK_EQUAL(LeakTest::count, 0);
}
