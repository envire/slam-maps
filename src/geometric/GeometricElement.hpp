#ifndef __MAPS_GEOMETRICELEMENT_HPP__
#define __MAPS_GEOMETRICELEMENT_HPP__

namespace maps
{
    class GeometricElementBase
    {
        public:
        template<class T>
        using PtrType = boost::shared_ptr<T>;

        typedef PtrType<GeometricElementBase> Ptr;


    };

    /**@brief GeometricElement class IEEE 1873 standard
     */
    template <typename _ElementData >
    class GeometricElement: public GeometricElementBase
    {
    public:
        typedef boost::shared_ptr< GeometricElement<_ElementData> > Ptr;
        typedef _ElementData TemplateType;

    protected:

        _ElementData user_data;

    public:

        GeometricElement() : GeometricElementBase()
        {
        }

        GeometricElement(const GeometricElement<_ElementData>& item) :  GeometricElementBase(item), user_data(item.user_data)
        {
        }

        GeometricElement(GeometricElement<_ElementData>&& item) :  GeometricElementBase(std::move(item)), user_data(std::move(item.user_data))
        {
        }

        template <typename... Ts>
        GeometricElement(Ts&&... args) : GeometricElementBase(), user_data(std::forward<Ts>(args)...)
        {
        }

        virtual ~GeometricElement() {}

        GeometricElement<_ElementData>& operator=(const GeometricElement<_ElementData>& item)
        {
            GeometricElementBase::operator=(item);
            user_data = item.user_data;
            return *this;
        }

        GeometricElement<_ElementData>& operator=(GeometricElement<_ElementData>&& item)
        {
            GeometricElementBase::operator=(std::move(item));
            user_data = std::move(item.user_data);
            return *this;
        }

        /**@brief setData
        *
        * Sets the user data
        *
        */
        void setData(const _ElementData& data) { this->user_data = data; }
        void setData(_ElementData&& data) { this->user_data = std::move(data); }

        /**@brief getData
        *
        * Returns the user data
        *
        */
        _ElementData& getData() { return this->user_data; }
        const _ElementData& getData() const { return this->user_data; }

        virtual std::type_index getTypeIndex() const
        {
            return std::type_index(typeid(maps::GeometricElement<_ElementData>));
        }

    };

}
#endif /* __MAPS_GEOMETRICELEMENT_HPP__ */
